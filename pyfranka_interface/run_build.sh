#!/bin/bash
set -e
rm -r build
python3 ./waf configure --python
python3 ./waf 
cp build/py*.so ./src/
echo "\n\n\n\n Don't forget to call:"
echo "export LD_LIBRARY_PATH=\$PWD/third_party/libfranka/lib:\$LD_LIBRARY_PATH"
#echo "export PATH=\$PATH:\$PWD/build"
python3 setup.py clean --all
python3 setup.py install