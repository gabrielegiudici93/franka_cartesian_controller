#!/usr/bin/env python3
"""
Train the best models for force regression, stretch classification, and location classification.

Uses optimal configurations:
- Fz regression: Normal forces only (20 per point), NO baseline removal
- Fx/Fy regression: Shear forces only (20 per point per direction)
- Location classification: Combined data (normal + shear), raw or magnitude features
- Stretch classification: Combined data (normal + shear)

Usage:
    python3 src/training/train_best_models.py \
        --normal-dir data/Multiple_Points/2.5mm_single_test42 \
        --shear-dir data/Multiple_Points/shear_forces_test51 \
        --run-label best_models \
        --remove-outliers
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List

import joblib
import numpy as np
from sklearn.metrics import accuracy_score, confusion_matrix, mean_squared_error, r2_score

CURRENT_DIR = Path(__file__).resolve().parent
SRC_ROOT = CURRENT_DIR.parent
REPO_ROOT = SRC_ROOT.parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from training.train_combined_shear_normal import (
    load_sequences_from_h5,
    remove_outliers,
    prepare_training_data,
    create_model,
    limit_sequences_per_location,
    limit_sequences_per_location_and_direction,
    identify_outliers,
)
from collections import defaultdict


def remove_first_and_outliers_per_location_direction(sequences: List[Dict], z_threshold: float = 3.0, remove_outliers: int = 2) -> List[Dict]:
    """Remove first sequence + N worst outliers per (location, direction) group for shear forces.
    
    CRITICAL: Ensures exactly 20 sequences per group remain for consistency.
    - If group has >= 23 sequences: remove 1 first + 2 worst outliers, then limit to 20
    - If group has < 23 but >= 20 sequences: limit to 20 WITHOUT removing outliers (consistency requirement)
    - If group has < 20 sequences: use all available (should not happen in normal data)
    """
    from collections import defaultdict
    
    # Group sequences by (location, direction)
    sequences_by_group = defaultdict(list)
    for idx, seq in enumerate(sequences):
        location = seq.get('offset', 'unknown')
        label = seq.get('label', '')
        if isinstance(label, bytes):
            label = label.decode('utf-8')
        
        # Determine direction
        direction = 'unknown'
        if 'shear' in label.lower():
            if 'x+' in label:
                direction = 'x+'
            elif 'x-' in label:
                direction = 'x-'
            elif 'y+' in label:
                direction = 'y+'
            elif 'y-' in label:
                direction = 'y-'
        
        key = (location, direction)
        sequences_by_group[key].append((idx, seq))
    
    all_indices_to_remove = set()
    target_sequences_per_group = 20
    
    for (location, direction), group_sequences in sequences_by_group.items():
        n_initial = len(group_sequences)
        
        if n_initial >= 23:
            # Standard case: remove first + 2 worst outliers, then limit to 20
            # Remove first sequence (index 0 in the group)
            first_idx = group_sequences[0][0]
            all_indices_to_remove.add(first_idx)
            
            # Get remaining sequences for outlier detection
            remaining_sequences = [seq for _, seq in group_sequences[1:]]
            
            # Identify outliers from remaining sequences
            outlier_indices_local = identify_outliers(remaining_sequences, z_threshold)
            
            # Calculate outlier scores for remaining sequences
            fz_values = [s.get('fz_max', 0) for s in remaining_sequences]
            duration_values = [s.get('duration', 0) for s in remaining_sequences]
            num_samples_values = [s.get('num_samples', 0) for s in remaining_sequences]
            
            fz_median = np.median(fz_values) if len(fz_values) > 0 else 0
            fz_mad = np.median(np.abs(np.array(fz_values) - fz_median)) if len(fz_values) > 0 and fz_median != 0 else 1.0
            duration_median = np.median(duration_values) if len(duration_values) > 0 else 0
            duration_mad = np.median(np.abs(np.array(duration_values) - duration_median)) if len(duration_values) > 0 and duration_median != 0 else 1.0
            num_samples_median = np.median(num_samples_values) if len(num_samples_values) > 0 else 0
            num_samples_mad = np.median(np.abs(np.array(num_samples_values) - num_samples_median)) if len(num_samples_values) > 0 and num_samples_median != 0 else 1.0
            
            remaining_with_scores = []
            for local_idx, (global_idx, seq) in enumerate(group_sequences[1:], start=1):
                fz = seq.get('fz_max', 0)
                duration = seq.get('duration', 0)
                num_samples = seq.get('num_samples', 0)
                
                score = 0
                if fz_mad > 0:
                    score += abs((fz - fz_median) / fz_mad)
                if duration_mad > 0:
                    duration_score = abs((duration - duration_median) / duration_mad)
                    if duration > duration_median * 2.0:
                        duration_score *= 2.0
                    score += duration_score * 1.5
                if num_samples_mad > 0:
                    score += abs((num_samples - num_samples_median) / num_samples_mad)
                
                is_identified_outlier = (local_idx - 1) in outlier_indices_local
                if is_identified_outlier:
                    score *= 2.0
                
                remaining_with_scores.append((global_idx, score, is_identified_outlier))
            
            # Sort by outlier status and score, take top N worst
            remaining_with_scores.sort(key=lambda x: (x[2], x[1]), reverse=True)
            num_to_remove = min(remove_outliers, len(remaining_with_scores))
            outliers_to_remove = [idx for idx, _, _ in remaining_with_scores[:num_to_remove]]
            
            all_indices_to_remove.update(outliers_to_remove)
            remaining_after_removal = n_initial - 1 - len(outliers_to_remove)
            
            # If still more than target, randomly remove excess to get exactly 20
            if remaining_after_removal > target_sequences_per_group:
                # Keep first sequence removed + outliers removed, then randomly sample to get exactly 20
                kept_indices = {first_idx} | set(outliers_to_remove)
                available_indices = [g[0] for g in group_sequences if g[0] not in kept_indices]
                np.random.seed(42)
                np.random.shuffle(available_indices)
                excess_to_remove = remaining_after_removal - target_sequences_per_group
                all_indices_to_remove.update(available_indices[:excess_to_remove])
                print(f"  Group ({location}, {direction}): {n_initial} initial -> removed 1 first + {len(outliers_to_remove)} outliers + {excess_to_remove} excess = {target_sequences_per_group} remaining")
            else:
                print(f"  Group ({location}, {direction}): {n_initial} initial -> removed 1 first + {len(outliers_to_remove)} outliers = {remaining_after_removal} remaining")
        
        elif n_initial >= target_sequences_per_group:
            # Group has 20-22 sequences: limit to 20 WITHOUT removing outliers (consistency requirement)
            np.random.seed(42)
            available_indices = [g[0] for g in group_sequences]
            np.random.shuffle(available_indices)
            excess_to_remove = n_initial - target_sequences_per_group
            indices_to_remove = available_indices[:excess_to_remove]
            all_indices_to_remove.update(indices_to_remove)
            print(f"  Group ({location}, {direction}): {n_initial} initial -> limited to {target_sequences_per_group} (NO outlier removal for consistency)")
        
        else:
            # Group has < 20 sequences: use all (should not happen, but handle gracefully)
            print(f"  Group ({location}, {direction}): WARNING - only {n_initial} sequences (less than target {target_sequences_per_group}), using all")
    
    cleaned_sequences = [s for i, s in enumerate(sequences) if i not in all_indices_to_remove]
    print(f"  Total removed: {len(all_indices_to_remove)} sequences")
    
    return cleaned_sequences


def split_sequences_by_location(
    sequences: List[Dict],
    actual_sequence_lengths: List[int],
    kept_sequence_indices: List[int],
    train_ratio: float = 0.7,
    random_seed: int = 42,
) -> tuple:
    """Split sequences separately for each location (70% train, 30% test per location)."""
    from collections import defaultdict
    
    # Only work with sequences that were kept after preprocessing
    kept_sequences = [sequences[orig_idx] for orig_idx in kept_sequence_indices]
    
    # Group sequences by location (using kept sequences only)
    sequences_by_location = defaultdict(list)
    for kept_seq_idx, seq in enumerate(kept_sequences):
        location = seq.get('offset', 'unknown')
        sequences_by_location[location].append(kept_seq_idx)
    
    # Split each location separately
    train_indices = []
    test_indices = []
    np.random.seed(random_seed)
    
    for location, seq_indices in sequences_by_location.items():
        n_seq = len(seq_indices)
        n_train = int(n_seq * train_ratio)
        
        # Shuffle indices for this location
        shuffled = np.random.permutation(seq_indices)
        train_indices.extend(shuffled[:n_train].tolist())
        test_indices.extend(shuffled[n_train:].tolist())
    
    # Map kept sequence indices to sample indices
    sequence_to_samples = {}
    current_idx = 0
    for kept_seq_idx, seq_len in enumerate(actual_sequence_lengths):
        sequence_to_samples[kept_seq_idx] = (current_idx, current_idx + seq_len)
        current_idx += seq_len
    
    train_samples = []
    test_samples = []
    for kept_seq_idx in train_indices:
        start, end = sequence_to_samples[kept_seq_idx]
        train_samples.extend(range(start, end))
    for kept_seq_idx in test_indices:
        start, end = sequence_to_samples[kept_seq_idx]
        test_samples.extend(range(start, end))
    
    return train_samples, test_samples, train_indices, test_indices


def split_sequences_by_location_and_direction(
    sequences: List[Dict],
    actual_sequence_lengths: List[int],
    kept_sequence_indices: List[int],
    train_ratio: float = 0.7,
    random_seed: int = 42,
) -> tuple:
    """Split sequences separately for each (location, direction) combination (70% train, 30% test per group)."""
    from collections import defaultdict
    
    # Only work with sequences that were kept after preprocessing
    kept_sequences = [sequences[orig_idx] for orig_idx in kept_sequence_indices]
    
    # Group sequences by location AND direction (using kept sequences only)
    sequences_by_group = defaultdict(list)
    for kept_seq_idx, seq in enumerate(kept_sequences):
        location = seq.get('offset', 'unknown')
        label = seq.get('label', '')
        if isinstance(label, bytes):
            label = label.decode('utf-8')
        
        # Determine direction
        direction = 'unknown'
        if 'shear' in label.lower():
            if 'x+' in label:
                direction = 'x+'
            elif 'x-' in label:
                direction = 'x-'
            elif 'y+' in label:
                direction = 'y+'
            elif 'y-' in label:
                direction = 'y-'
        else:
            direction = 'normal'  # For normal forces
        
        key = (location, direction)
        sequences_by_group[key].append(kept_seq_idx)
    
    # Split each group separately
    train_indices = []
    test_indices = []
    np.random.seed(random_seed)
    
    for (location, direction), seq_indices in sequences_by_group.items():
        n_seq = len(seq_indices)
        n_train = int(n_seq * train_ratio)
        
        # Shuffle indices for this group
        shuffled = np.random.permutation(seq_indices)
        train_indices.extend(shuffled[:n_train].tolist())
        test_indices.extend(shuffled[n_train:].tolist())
    
    # Map kept sequence indices to sample indices
    sequence_to_samples = {}
    current_idx = 0
    for kept_seq_idx, seq_len in enumerate(actual_sequence_lengths):
        sequence_to_samples[kept_seq_idx] = (current_idx, current_idx + seq_len)
        current_idx += seq_len
    
    train_samples = []
    test_samples = []
    for kept_seq_idx in train_indices:
        start, end = sequence_to_samples[kept_seq_idx]
        train_samples.extend(range(start, end))
    for kept_seq_idx in test_indices:
        start, end = sequence_to_samples[kept_seq_idx]
        test_samples.extend(range(start, end))
    
    return train_samples, test_samples, train_indices, test_indices


def train_fz_regressor(
    sequences_by_stretch: Dict[str, List[Dict]],
    train_ratio: float = 0.7,
) -> Dict:
    """Train Fz regressor using normal forces only (best configuration)."""
    print(f"\n{'='*80}")
    print("TRAINING FZ REGRESSOR (Normal Forces Only)")
    print(f"{'='*80}")
    
    all_sequences = []
    for stretch_label_key, sequences in sequences_by_stretch.items():
        for seq in sequences:
            seq['stretch_label'] = stretch_label_key
        all_sequences.extend(sequences)
    
    print(f"Total sequences: {len(all_sequences)}")
    
    # Prepare data - NO baseline removal for Fz
    X, y_fx, y_fy, y_fz, y_offset, scaler, fz_scaler, actual_sequence_lengths, kept_sequence_indices = prepare_training_data(
        all_sequences,
        normalize=True,
        use_feature_engineering=False,
        filter_displacement=True,
        displacement_threshold=95.0,
        normalize_fz=False,
        fz_target_min=0.0,
        fz_target_max=3.0,
        include_offset_labels=False,
        use_advanced_features=False,
        location_feature_method='raw',
        remove_fz_baseline=False  # NO baseline removal for Fz
    )
    
    print(f"Total samples: {len(X)}")
    print(f"Features: {X.shape[1]}")
    print(f"Fz range: [{np.min(y_fz):.3f}, {np.max(y_fz):.3f}] N")
    
    # Split by location (70% train, 30% test per location)
    train_samples, test_samples, train_indices, test_indices = split_sequences_by_location(
        all_sequences, actual_sequence_lengths, kept_sequence_indices, train_ratio=train_ratio, random_seed=42
    )
    
    X_train = X[train_samples]
    X_test = X[test_samples]
    y_fz_train = y_fz[train_samples]
    y_fz_test = y_fz[test_samples]
    
    print(f"\nTrain: {len(train_indices)} sequences, {len(X_train)} samples")
    print(f"Test: {len(test_indices)} sequences, {len(X_test)} samples")
    
    # Train Fz regressor
    print("\nTraining Fz regressor...")
    fz_model = create_model(regressor=True, use_gpu=False, n_estimators=200, gpu_id=0)
    fz_model.fit(X_train, y_fz_train)
    y_fz_pred = fz_model.predict(X_test)
    
    rmse = np.sqrt(mean_squared_error(y_fz_test, y_fz_pred))
    r2 = r2_score(y_fz_test, y_fz_pred)
    
    print(f"  Fz - Train RMSE: {np.sqrt(mean_squared_error(y_fz_train, fz_model.predict(X_train))):.4f} N")
    print(f"  Fz - Test RMSE: {rmse:.4f} N")
    print(f"  Fz - R²: {r2:.4f}")
    
    return {
        'fz_model': fz_model,
        'scaler': scaler,
        'rmse_fz': rmse,
        'r2_fz': r2,
        'y_fz_test': y_fz_test,
        'y_fz_pred': y_fz_pred,
        'n_train': len(train_indices),
        'n_test': len(test_indices),
    }


def train_fx_fy_regressors(
    sequences_by_stretch: Dict[str, List[Dict]],
    train_ratio: float = 0.7,
) -> Dict:
    """Train Fx, Fy regressors using shear forces only (best configuration)."""
    print(f"\n{'='*80}")
    print("TRAINING FX, FY REGRESSORS (Shear Forces Only)")
    print(f"{'='*80}")
    
    # Separate sequences by direction
    fx_sequences = []
    fy_sequences = []
    
    for stretch_label_key, sequences in sequences_by_stretch.items():
        for seq in sequences:
            seq['stretch_label'] = stretch_label_key
            label = seq.get('label', '')
            if isinstance(label, bytes):
                label = label.decode('utf-8')
            
            if 'shear' in label.lower():
                if 'x+' in label or 'x-' in label:
                    fx_sequences.append(seq)
                elif 'y+' in label or 'y-' in label:
                    fy_sequences.append(seq)
    
    print(f"Total sequences for Fx regression (x+ and x-): {len(fx_sequences)}")
    print(f"Total sequences for Fy regression (y+ and y-): {len(fy_sequences)}")
    
    # Prepare data for Fx regression
    X_fx, y_fx_all, y_fy_dummy, y_fz_dummy, y_offset_dummy, scaler_fx, fz_scaler_dummy, actual_sequence_lengths_fx, kept_sequence_indices_fx = prepare_training_data(
        fx_sequences,
        normalize=True,
        use_feature_engineering=False,
        filter_displacement=True,
        displacement_threshold=95.0,
        normalize_fz=False,
        fz_target_min=0.0,
        fz_target_max=3.0,
        include_offset_labels=False,
        use_advanced_features=False,
        location_feature_method='raw'
    )
    
    # Prepare data for Fy regression
    X_fy, y_fx_dummy, y_fy_all, y_fz_dummy, y_offset_dummy, scaler_fy, fz_scaler_dummy, actual_sequence_lengths_fy, kept_sequence_indices_fy = prepare_training_data(
        fy_sequences,
        normalize=True,
        use_feature_engineering=False,
        filter_displacement=True,
        displacement_threshold=95.0,
        normalize_fz=False,
        fz_target_min=0.0,
        fz_target_max=3.0,
        include_offset_labels=False,
        use_advanced_features=False,
        location_feature_method='raw'
    )
    
    print(f"\nFx regression data: {len(X_fx)} samples")
    print(f"Fy regression data: {len(X_fy)} samples")
    
    # Split Fx sequences by (location, direction) - 70% train, 30% test per group
    train_samples_fx, test_samples_fx, train_indices_fx, test_indices_fx = split_sequences_by_location_and_direction(
        fx_sequences, actual_sequence_lengths_fx, kept_sequence_indices_fx, train_ratio=train_ratio, random_seed=42
    )
    
    X_fx_train = X_fx[train_samples_fx]
    X_fx_test = X_fx[test_samples_fx]
    y_fx_train = y_fx_all[train_samples_fx]
    y_fx_test = y_fx_all[test_samples_fx]
    
    print(f"\nFx - Train: {len(train_indices_fx)} sequences, {len(X_fx_train)} samples")
    print(f"Fx - Test: {len(test_indices_fx)} sequences, {len(X_fx_test)} samples")
    
    # Train Fx regressor
    print("\nTraining Fx regressor...")
    fx_model = create_model(regressor=True, use_gpu=False, n_estimators=200, gpu_id=0)
    fx_model.fit(X_fx_train, y_fx_train)
    y_fx_pred = fx_model.predict(X_fx_test)
    
    rmse_fx = np.sqrt(mean_squared_error(y_fx_test, y_fx_pred))
    r2_fx = r2_score(y_fx_test, y_fx_pred)
    
    print(f"  Fx - Train RMSE: {np.sqrt(mean_squared_error(y_fx_train, fx_model.predict(X_fx_train))):.4f} N")
    print(f"  Fx - Test RMSE: {rmse_fx:.4f} N")
    print(f"  Fx - R²: {r2_fx:.4f}")
    
    # Split Fy sequences by (location, direction) - 70% train, 30% test per group
    train_samples_fy, test_samples_fy, train_indices_fy, test_indices_fy = split_sequences_by_location_and_direction(
        fy_sequences, actual_sequence_lengths_fy, kept_sequence_indices_fy, train_ratio=train_ratio, random_seed=42
    )
    
    X_fy_train = X_fy[train_samples_fy]
    X_fy_test = X_fy[test_samples_fy]
    y_fy_train = y_fy_all[train_samples_fy]
    y_fy_test = y_fy_all[test_samples_fy]
    
    print(f"\nFy - Train: {len(train_indices_fy)} sequences, {len(X_fy_train)} samples")
    print(f"Fy - Test: {len(test_indices_fy)} sequences, {len(X_fy_test)} samples")
    
    # Train Fy regressor
    print("\nTraining Fy regressor...")
    fy_model = create_model(regressor=True, use_gpu=False, n_estimators=200, gpu_id=0)
    fy_model.fit(X_fy_train, y_fy_train)
    y_fy_pred = fy_model.predict(X_fy_test)
    
    rmse_fy = np.sqrt(mean_squared_error(y_fy_test, y_fy_pred))
    r2_fy = r2_score(y_fy_test, y_fy_pred)
    
    print(f"  Fy - Train RMSE: {np.sqrt(mean_squared_error(y_fy_train, fy_model.predict(X_fy_train))):.4f} N")
    print(f"  Fy - Test RMSE: {rmse_fy:.4f} N")
    print(f"  Fy - R²: {r2_fy:.4f}")
    
    return {
        'fx_model': fx_model,
        'fy_model': fy_model,
        'scaler_fx': scaler_fx,
        'scaler_fy': scaler_fy,
        'rmse_fx': rmse_fx,
        'rmse_fy': rmse_fy,
        'r2_fx': r2_fx,
        'r2_fy': r2_fy,
        'y_fx_test': y_fx_test,
        'y_fx_pred': y_fx_pred,
        'y_fy_test': y_fy_test,
        'y_fy_pred': y_fy_pred,
        'n_train_fx': len(train_indices_fx),
        'n_test_fx': len(test_indices_fx),
        'n_train_fy': len(train_indices_fy),
        'n_test_fy': len(test_indices_fy),
    }


def train_location_classifier(
    sequences_by_stretch: Dict[str, List[Dict]],
    train_ratio: float = 0.7,
    location_feature_method: str = 'raw',
) -> Dict:
    """Train location classifier using combined data (best configuration)."""
    print(f"\n{'='*80}")
    print(f"TRAINING LOCATION CLASSIFIER ({location_feature_method} features)")
    print(f"{'='*80}")
    
    all_sequences = []
    for stretch_label_key, sequences in sequences_by_stretch.items():
        for seq in sequences:
            seq['stretch_label'] = stretch_label_key
        all_sequences.extend(sequences)
    
    print(f"Total sequences: {len(all_sequences)}")
    
    # Prepare data
    X, y_fx, y_fy, y_fz, y_offset, scaler, fz_scaler, actual_sequence_lengths, kept_sequence_indices = prepare_training_data(
        all_sequences,
        normalize=True,
        use_feature_engineering=False,
        filter_displacement=True,
        displacement_threshold=95.0,
        normalize_fz=False,
        fz_target_min=0.0,
        fz_target_max=3.0,
        include_offset_labels=False,
        use_advanced_features=False,
        location_feature_method=location_feature_method,
        remove_fz_baseline=False
    )
    
    print(f"Total samples: {len(X)}")
    print(f"Features: {X.shape[1]}")
    print(f"Kept sequences after preprocessing: {len(kept_sequence_indices)}")
    
    # Group sequences by (location, direction) AFTER preprocessing
    # We need to ensure all groups have exactly 20 sequences
    from collections import defaultdict
    kept_sequences_by_group = defaultdict(list)
    for kept_seq_idx, orig_idx in enumerate(kept_sequence_indices):
        seq = all_sequences[orig_idx]
        location = seq.get('offset', 'unknown')
        label = seq.get('label', '')
        if isinstance(label, bytes):
            label = label.decode('utf-8')
        
        # Determine direction
        direction = 'unknown'
        if 'shear' in label.lower():
            if 'x+' in label:
                direction = 'x+'
            elif 'x-' in label:
                direction = 'x-'
            elif 'y+' in label:
                direction = 'y+'
            elif 'y-' in label:
                direction = 'y-'
        else:
            direction = 'normal'
        
        key = (location, direction)
        kept_sequences_by_group[key].append((kept_seq_idx, orig_idx))
    
    # Find minimum number of sequences per group
    min_sequences_per_group = min(len(indices) for indices in kept_sequences_by_group.values()) if kept_sequences_by_group else 0
    max_sequences_per_group = max(len(indices) for indices in kept_sequences_by_group.values()) if kept_sequences_by_group else 0
    
    print(f"\nSequences per group after preprocessing:")
    print(f"  Minimum: {min_sequences_per_group}")
    print(f"  Maximum: {max_sequences_per_group}")
    
    # Check if any group has less than 20 sequences
    groups_with_less_than_20 = [(key, len(indices)) for key, indices in kept_sequences_by_group.items() if len(indices) < 20]
    if groups_with_less_than_20:
        print(f"\nWARNING: Found {len(groups_with_less_than_20)} groups with less than 20 sequences:")
        for key, count in sorted(groups_with_less_than_20):
            print(f"  {key}: {count} sequences")
    
    # Limit all groups to exactly 20 sequences
    # This ensures consistency across all groups
    target_sequences_per_group = 20
    if min_sequences_per_group < 20:
        print(f"\nERROR: Cannot limit all groups to 20 sequences. Minimum is {min_sequences_per_group}.")
        print(f"Some groups will have less than 20 sequences.")
        target_sequences_per_group = min_sequences_per_group
    else:
        print(f"\nLimiting all groups to exactly {target_sequences_per_group} sequences for consistency.")
    
    # Create filtered kept_sequence_indices and actual_sequence_lengths
    filtered_kept_indices = []  # These are kept_seq_idx (position in kept_sequence_indices)
    filtered_orig_indices = []  # These are orig_idx (position in all_sequences)
    np.random.seed(42)  # Use same seed for reproducibility
    
    for key, group_items in kept_sequences_by_group.items():
        # Shuffle and take exactly target_sequences_per_group sequences
        shuffled = np.random.permutation(group_items)
        selected = shuffled[:target_sequences_per_group]
        for kept_seq_idx, orig_idx in selected:
            filtered_kept_indices.append(kept_seq_idx)
            filtered_orig_indices.append(orig_idx)
    
    # Sort by kept_seq_idx to maintain order
    sorted_pairs = sorted(zip(filtered_kept_indices, filtered_orig_indices))
    filtered_kept_indices = [k for k, o in sorted_pairs]
    filtered_orig_indices = [o for k, o in sorted_pairs]
    
    # Update kept_sequence_indices and actual_sequence_lengths
    filtered_kept_sequence_indices = filtered_orig_indices
    filtered_actual_sequence_lengths = [actual_sequence_lengths[k] for k in filtered_kept_indices]
    
    # Rebuild X, y_offset arrays to match filtered sequences
    # Map old sample indices to new sample indices
    sample_start = 0
    new_X_parts = []
    new_y_offset_parts = []
    
    for kept_seq_idx in filtered_kept_indices:
        # Find start/end in original X array
        start_idx = sum(actual_sequence_lengths[:kept_seq_idx])
        seq_len = actual_sequence_lengths[kept_seq_idx]
        end_idx = start_idx + seq_len
        
        new_X_parts.append(X[start_idx:end_idx])
        new_y_offset_parts.append(y_offset[start_idx:end_idx])
    
    X = np.vstack(new_X_parts) if new_X_parts else np.array([])
    y_offset = np.concatenate(new_y_offset_parts) if new_y_offset_parts else np.array([])
    actual_sequence_lengths = filtered_actual_sequence_lengths
    kept_sequence_indices = filtered_kept_sequence_indices
    
    print(f"After limiting to {target_sequences_per_group} sequences per group:")
    print(f"  Total sequences: {len(kept_sequence_indices)}")
    print(f"  Total samples: {len(X)}")
    
    # Split by (location, direction) - 70% train, 30% test per group
    train_samples, test_samples, train_indices, test_indices = split_sequences_by_location_and_direction(
        all_sequences, actual_sequence_lengths, kept_sequence_indices, train_ratio=train_ratio, random_seed=42
    )
    
    X_train = X[train_samples]
    X_test = X[test_samples]
    y_offset_train = y_offset[train_samples]
    y_offset_test = y_offset[test_samples]
    
    print(f"\nTrain: {len(train_indices)} sequences, {len(X_train)} samples")
    print(f"Test: {len(test_indices)} sequences, {len(X_test)} samples")
    
    # Train location classifier
    print("\nTraining location classifier...")
    location_model = create_model(regressor=False, use_gpu=False, n_estimators=200, gpu_id=0)
    location_model.fit(X_train, y_offset_train)
    y_offset_pred = location_model.predict(X_test)
    
    accuracy = accuracy_score(y_offset_test, y_offset_pred)
    cm = confusion_matrix(y_offset_test, y_offset_pred, labels=np.arange(11))
    
    print(f"  Location - Train Accuracy: {accuracy_score(y_offset_train, location_model.predict(X_train)):.4f}")
    print(f"  Location - Test Accuracy: {accuracy:.4f}")
    
    return {
        'location_model': location_model,
        'scaler': scaler,
        'accuracy': accuracy,
        'confusion_matrix': cm.tolist(),
        'n_train': len(train_indices),
        'n_test': len(test_indices),
        'y_offset_test': y_offset_test,
        'y_offset_pred': y_offset_pred,
    }


def train_stretch_classifier(
    sequences_by_stretch: Dict[str, List[Dict]],
    train_ratio: float = 0.7,
    location_feature_method: str = 'raw',
) -> Dict:
    """Train stretch classifier using combined data."""
    print(f"\n{'='*80}")
    print(f"TRAINING STRETCH CLASSIFIER ({location_feature_method} features)")
    print(f"{'='*80}")
    
    all_sequences = []
    all_stretches = []
    for stretch_label_key, sequences in sequences_by_stretch.items():
        for seq in sequences:
            seq['stretch_label'] = stretch_label_key
        all_sequences.extend(sequences)
        all_stretches.extend([stretch_label_key] * len(sequences))
    
    print(f"Total sequences: {len(all_sequences)}")
    
    # Prepare data
    X, y_fx, y_fy, y_fz, y_offset, scaler, fz_scaler, actual_sequence_lengths, kept_sequence_indices = prepare_training_data(
        all_sequences,
        normalize=True,
        use_feature_engineering=False,
        filter_displacement=True,
        displacement_threshold=95.0,
        normalize_fz=False,
        fz_target_min=0.0,
        fz_target_max=3.0,
        include_offset_labels=False,
        use_advanced_features=False,
        location_feature_method=location_feature_method,
        remove_fz_baseline=False
    )
    
    print(f"Total samples: {len(X)}")
    print(f"Features: {X.shape[1]}")
    
    # Encode stretch labels (using kept sequences only)
    stretch_map = {'000pct': 0, '010pct': 1, '020pct': 2}
    y_stretch = []
    for kept_seq_idx, seq_len in enumerate(actual_sequence_lengths):
        orig_seq_idx = kept_sequence_indices[kept_seq_idx]
        stretch_label = all_stretches[orig_seq_idx]
        stretch_num = stretch_map.get(stretch_label, 0)
        y_stretch.extend([stretch_num] * seq_len)
    y_stretch = np.array(y_stretch)
    
    # Split by (location, direction) - 70% train, 30% test per group
    train_samples, test_samples, train_indices, test_indices = split_sequences_by_location_and_direction(
        all_sequences, actual_sequence_lengths, kept_sequence_indices, train_ratio=train_ratio, random_seed=42
    )
    
    X_train = X[train_samples]
    X_test = X[test_samples]
    y_stretch_train = y_stretch[train_samples]
    y_stretch_test = y_stretch[test_samples]
    
    print(f"\nTrain: {len(train_indices)} sequences, {len(X_train)} samples")
    print(f"Test: {len(test_indices)} sequences, {len(X_test)} samples")
    
    # Train stretch classifier
    print("\nTraining stretch classifier...")
    stretch_model = create_model(regressor=False, use_gpu=False, n_estimators=200, gpu_id=0)
    stretch_model.fit(X_train, y_stretch_train)
    y_stretch_pred = stretch_model.predict(X_test)
    
    accuracy = accuracy_score(y_stretch_test, y_stretch_pred)
    cm = confusion_matrix(y_stretch_test, y_stretch_pred, labels=np.arange(3))
    
    print(f"  Stretch - Train Accuracy: {accuracy_score(y_stretch_train, stretch_model.predict(X_train)):.4f}")
    print(f"  Stretch - Test Accuracy: {accuracy:.4f}")
    
    return {
        'stretch_model': stretch_model,
        'scaler': scaler,
        'accuracy': accuracy,
        'confusion_matrix': cm.tolist(),
        'n_train': len(train_indices),
        'n_test': len(test_indices),
        'y_stretch_test': y_stretch_test,
        'y_stretch_pred': y_stretch_pred,
    }


def main():
    parser = argparse.ArgumentParser(description="Train best models for all tasks")
    parser.add_argument('--normal-dir', type=Path, required=True, help='Directory with normal forces HDF5 files')
    parser.add_argument('--shear-dir', type=Path, required=True, help='Directory with shear forces HDF5 files')
    parser.add_argument('--run-label', type=str, default='best_models', help='Run label for output')
    parser.add_argument('--remove-outliers', action='store_true', help='Remove outliers')
    parser.add_argument('--z-threshold', type=float, default=3.0, help='Z-score threshold for outliers')
    
    args = parser.parse_args()
    
    # Find normal forces files
    normal_files_dict = {}
    for stretch in ['000pct', '010pct', '020pct']:
        pattern = f"*stretch_{stretch}.h5"
        files = list(args.normal_dir.glob(pattern))
        files = [f for f in files if 'no_touch' not in f.name]
        if files:
            normal_files_dict[stretch] = files[0]
            print(f"Found NORMAL {stretch}: {files[0].name}")
    
    # Find shear forces files
    shear_files_dict = {}
    for stretch in ['000pct', '010pct', '020pct']:
        pattern = f"*shear*stretch_{stretch}.h5"
        files = list(args.shear_dir.glob(pattern))
        if files:
            shear_files_dict[stretch] = files[0]
            print(f"Found SHEAR {stretch}: {files[0].name}")
    
    # Load normal forces sequences
    print(f"\n{'='*80}")
    print("LOADING NORMAL FORCES (for Fz regression)")
    print(f"{'='*80}")
    normal_sequences = {}
    for stretch, h5_file in normal_files_dict.items():
        sequences = load_sequences_from_h5(h5_file)
        for seq in sequences:
            seq['stretch_label'] = stretch
        
        if args.remove_outliers:
            sequences, _ = remove_outliers(sequences, z_threshold=args.z_threshold, remove_per_offset=2)
        
        sequences = limit_sequences_per_location(sequences, max_sequences_per_location=20, random_seed=42)
        normal_sequences[stretch] = sequences
        print(f"  {stretch}: {len(sequences)} sequences (20 per location)")
    
    # Load shear forces sequences
    print(f"\n{'='*80}")
    print("LOADING SHEAR FORCES (for Fx/Fy regression and classification)")
    print(f"{'='*80}")
    shear_sequences = {}
    for stretch, h5_file in shear_files_dict.items():
        sequences = load_sequences_from_h5(h5_file)
        for seq in sequences:
            seq['stretch_label'] = stretch
        
        # For shear forces: remove exactly 3 sequences per (location, direction) group:
        # 1 first sequence + 2 worst outliers = 20 sequences per group (from 23 total)
        if args.remove_outliers:
            sequences = remove_first_and_outliers_per_location_direction(sequences, z_threshold=args.z_threshold, remove_outliers=2)
        
        sequences = limit_sequences_per_location_and_direction(sequences, max_sequences_per_location_per_direction=20, random_seed=42)
        shear_sequences[stretch] = sequences
        print(f"  {stretch}: {len(sequences)} sequences (20 per location per direction)")
    
    # Combine for classification tasks
    combined_sequences = {}
    for stretch in ['000pct', '010pct', '020pct']:
        combined = []
        if stretch in normal_sequences:
            combined.extend(normal_sequences[stretch])
        if stretch in shear_sequences:
            combined.extend(shear_sequences[stretch])
        combined_sequences[stretch] = combined
        print(f"  {stretch}: {len(combined)} combined sequences")
    
    # Create output directory
    output_dir = args.shear_dir / "best_models" / args.run_label
    models_dir = output_dir / "models"
    models_dir.mkdir(parents=True, exist_ok=True)
    
    # Train all models
    print(f"\n{'='*80}")
    print("TRAINING ALL MODELS")
    print(f"{'='*80}")
    
    # 1. Force regression - PER STRETCH FIRST
    print(f"\n{'='*80}")
    print("TRAINING PER-STRETCH MODELS")
    print(f"{'='*80}")
    
    fz_per_stretch = {}
    for stretch_label, sequences in normal_sequences.items():
        print(f"\nTraining Fz regressor for {stretch_label}...")
        result = train_fz_regressor({stretch_label: sequences}, train_ratio=0.7)
        fz_per_stretch[stretch_label] = {
            'rmse': result['rmse_fz'],
            'r2': result['r2_fz'],
            'n_train': result['n_train'],
            'n_test': result['n_test'],
        }
    
    fx_fy_per_stretch = {'fx': {}, 'fy': {}}
    for stretch_label, sequences in shear_sequences.items():
        print(f"\nTraining Fx/Fy regressors for {stretch_label}...")
        result = train_fx_fy_regressors({stretch_label: sequences}, train_ratio=0.7)
        fx_fy_per_stretch['fx'][stretch_label] = {
            'rmse': result['rmse_fx'],
            'r2': result['r2_fx'],
            'n_train': result['n_train_fx'],
            'n_test': result['n_test_fx'],
        }
        fx_fy_per_stretch['fy'][stretch_label] = {
            'rmse': result['rmse_fy'],
            'r2': result['r2_fy'],
            'n_train': result['n_train_fy'],
            'n_test': result['n_test_fy'],
        }
    
    location_per_stretch = {}
    for stretch_label, sequences in combined_sequences.items():
        print(f"\nTraining location classifier for {stretch_label}...")
        result = train_location_classifier({stretch_label: sequences}, train_ratio=0.7)
        location_per_stretch[stretch_label] = {
            'accuracy': result['accuracy'],
            'n_train': result['n_train'],
            'n_test': result['n_test'],
        }
    
    # 2. Force regression - COMBINED
    print(f"\n{'='*80}")
    print("TRAINING COMBINED MODELS")
    print(f"{'='*80}")
    
    fz_result = train_fz_regressor(normal_sequences, train_ratio=0.7)
    fx_fy_result = train_fx_fy_regressors(shear_sequences, train_ratio=0.7)
    
    # 3. Location classification (test both raw and magnitude)
    location_results = {}
    for method in ['raw', 'magnitude']:
        location_results[method] = train_location_classifier(combined_sequences, train_ratio=0.7, location_feature_method=method)
    
    # 3. Stretch classification (use raw features)
    stretch_result = train_stretch_classifier(combined_sequences, train_ratio=0.7, location_feature_method='raw')
    
    # Save models
    print(f"\n{'='*80}")
    print("SAVING MODELS")
    print(f"{'='*80}")
    
    joblib.dump(fz_result['fz_model'], models_dir / "fz_regressor.joblib")
    print(f"  ✓ Saved: fz_regressor.joblib")
    
    joblib.dump(fx_fy_result['fx_model'], models_dir / "fx_regressor.joblib")
    print(f"  ✓ Saved: fx_regressor.joblib")
    
    joblib.dump(fx_fy_result['fy_model'], models_dir / "fy_regressor.joblib")
    print(f"  ✓ Saved: fy_regressor.joblib")
    
    # Save best location classifier (choose between raw and magnitude)
    best_location_method = max(location_results.items(), key=lambda x: x[1]['accuracy'])[0]
    joblib.dump(location_results[best_location_method]['location_model'], models_dir / "location_classifier.joblib")
    joblib.dump(location_results[best_location_method]['scaler'], models_dir / "location_scaler.joblib")
    print(f"  ✓ Saved: location_classifier.joblib ({best_location_method} features)")
    
    joblib.dump(stretch_result['stretch_model'], models_dir / "stretch_classifier.joblib")
    joblib.dump(stretch_result['scaler'], models_dir / "stretch_scaler.joblib")
    print(f"  ✓ Saved: stretch_classifier.joblib")
    
    # Save scalers for force regression
    joblib.dump(fz_result['scaler'], models_dir / "scaler_fz.joblib")
    joblib.dump(fx_fy_result['scaler_fx'], models_dir / "scaler_fx.joblib")
    joblib.dump(fx_fy_result['scaler_fy'], models_dir / "scaler_fy.joblib")
    
    # Save metrics
    metrics = {
        'run_label': args.run_label,
        'force_regression': {
            'fz': {
                'per_stretch': fz_per_stretch,
                'combined': {
                    'rmse': float(fz_result['rmse_fz']),
                    'r2': float(fz_result['r2_fz']),
                    'n_train': fz_result['n_train'],
                    'n_test': fz_result['n_test'],
                    'dataset': 'normal_forces_only',
                    'note': 'NO baseline removal',
                }
            },
            'fx': {
                'per_stretch': fx_fy_per_stretch['fx'],
                'combined': {
                    'rmse': float(fx_fy_result['rmse_fx']),
                    'r2': float(fx_fy_result['r2_fx']),
                    'n_train': fx_fy_result['n_train_fx'],
                    'n_test': fx_fy_result['n_test_fx'],
                    'dataset': 'shear_forces_only_x_directions',
                }
            },
            'fy': {
                'per_stretch': fx_fy_per_stretch['fy'],
                'combined': {
                    'rmse': float(fx_fy_result['rmse_fy']),
                    'r2': float(fx_fy_result['r2_fy']),
                    'n_train': fx_fy_result['n_train_fy'],
                    'n_test': fx_fy_result['n_test_fy'],
                    'dataset': 'shear_forces_only_y_directions',
                }
            },
        },
        'location_classification': {
            'per_stretch': location_per_stretch,
            'combined': {
                method: {
                    'accuracy': float(result['accuracy']),
                    'n_train': result['n_train'],
                    'n_test': result['n_test'],
                }
                for method, result in location_results.items()
            },
            'best_location_method': best_location_method,
        },
        'stretch_classification': {
            'accuracy': float(stretch_result['accuracy']),
            'n_train': stretch_result['n_train'],
            'n_test': stretch_result['n_test'],
            'feature_method': 'raw',
        },
    }
    
    metrics_file = output_dir / "metrics.json"
    with open(metrics_file, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f"\n  ✓ Saved metrics: {metrics_file}")
    
    # Print summary
    print(f"\n{'='*80}")
    print("FINAL RESULTS SUMMARY")
    print(f"{'='*80}")
    
    print("\n📍 FORCE REGRESSION:")
    print(f"   Fz: RMSE {metrics['force_regression']['fz']['combined']['rmse']:.4f} N, R² {metrics['force_regression']['fz']['combined']['r2']:.4f}")
    print(f"   Fx: RMSE {metrics['force_regression']['fx']['combined']['rmse']:.4f} N, R² {metrics['force_regression']['fx']['combined']['r2']:.4f}")
    print(f"   Fy: RMSE {metrics['force_regression']['fy']['combined']['rmse']:.4f} N, R² {metrics['force_regression']['fy']['combined']['r2']:.4f}")
    
    print("\n📍 LOCATION CLASSIFICATION:")
    if 'combined' in metrics['location_classification']:
        for method, result in metrics['location_classification']['combined'].items():
            print(f"   {method:20s}: {result['accuracy']:.4f} ({result['accuracy']*100:.2f}%)")
        print(f"   ✅ Best method: {best_location_method} (accuracy: {metrics['location_classification']['combined'][best_location_method]['accuracy']:.4f})")
    else:
        for method, result in metrics['location_classification'].items():
            if isinstance(result, dict) and 'accuracy' in result:
                print(f"   {method:20s}: {result['accuracy']:.4f} ({result['accuracy']*100:.2f}%)")
    
    print("\n📍 STRETCH CLASSIFICATION:")
    print(f"   Accuracy: {metrics['stretch_classification']['accuracy']:.4f} ({metrics['stretch_classification']['accuracy']*100:.2f}%)")
    
    print(f"\n{'='*80}")
    print("✓ Training complete!")
    print(f"  Models: {models_dir}")
    print(f"  Metrics: {metrics_file}")
    print(f"{'='*80}")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

