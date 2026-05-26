# Git Workflow (Commit & Push)

Daily commands for updating the repo, saving your changes, and publishing to GitHub.

**Repository:** `https://github.com/gabrielegiudici93/franka_cartesian_controller`  
**Recommended remote:** SSH (`git@github.com:gabrielegiudici93/franka_cartesian_controller.git`)

---

## 1. Get the latest code (before you work)

```bash
cd ~/franka_cartesian_controller
git pull --rebase origin main
```

Use this every time you start work, and again **before** you push your own commits.

---

## 2. Check what changed

```bash
cd ~/franka_cartesian_controller
git status
git diff
```

---

## 3. Commit your changes

```bash
cd ~/franka_cartesian_controller

# Stage files (all changes)
git add .

# Or stage specific paths only
# git add magtec_models/src/franka_controller/teleop_franka_keyboard.py
# git add magtec_models/docs/guides/DATA_COLLECTION.md

git commit -m "Short description of what you changed and why"
```

**Example commit messages:**

```text
docs: update data collection guide with teleop link
fix: teleop GUI compatibility with 10-point predictor
feat: add new stretch level to collection script
```

---

## 4. Push to GitHub

```bash
cd ~/franka_cartesian_controller
git pull --rebase origin main
git push origin main
```

If push is rejected (“fetch first” or divergent branches), run `git pull --rebase origin main` again, fix any conflicts, then `git push origin main`.

---

## One-shot copy/paste (typical session)

```bash
cd ~/franka_cartesian_controller
git pull --rebase origin main

# ... edit files, test scripts ...

git status
git add .
git commit -m "docs: describe your change here"
git pull --rebase origin main
git push origin main
```

---

## SSH vs HTTPS

### SSH (recommended on this machine)

Remote should look like:

```bash
git remote -v
# origin  git@github.com:gabrielegiudici93/franka_cartesian_controller.git
```

If you still have HTTPS:

```bash
git remote set-url origin git@github.com:gabrielegiudici93/franka_cartesian_controller.git
git push origin main
```

Test SSH access:

```bash
ssh -T git@github.com
```

### HTTPS (Personal Access Token)

If you use `https://github.com/...`:

- **Username:** your GitHub username  
- **Password:** a [Personal Access Token](https://github.com/settings/tokens) (`repo` scope), **not** your GitHub account password  

Cursor/VS Code credential helpers may fail in the terminal (`vscode-git-*.sock`). Use an external terminal or SSH instead.

---

## What not to commit

Already in `.gitignore` (do not force-add):

- `magtec_models/config/hardware.yaml` (machine-specific)
- `magtec_models/data/**/*.h5`, `models/**`, `plots/**`, `logs/**`
- `pyfranka_interface/build/`, `__pycache__/`, `*.so`

---

## First-time clone (new collaborator)

```bash
git clone git@github.com:gabrielegiudici93/franka_cartesian_controller.git
cd franka_cartesian_controller
conda env create -f environment.yml
# then follow docs/INSTALL.md
```

---

## Related

- [Initial GitHub setup](SETUP_GITHUB.md) — first publish, LFS, vendored deps
- [Data collection guide](../magtec_models/docs/guides/DATA_COLLECTION.md)
- [Teleoperation guide](../magtec_models/docs/guides/TELEOPERATION.md)
