#!/bin/bash
# Tiny probe (Mac Intel): the macos-15 runner uses Apple's NEW linker
# (ld-1167.x / "ld_prime") which is strict and won't resolve isis symbols the
# way the classic ld64 (lenient, transitive) does. Test which linker/flag makes
# an explicit isis link succeed with -undefined,error. Builds NOTHING big.
set +e

isArm64=$(uname -m | grep arm64)
if [ "$isArm64" != "" ]; then echo "PROBE: arm not the subject; exit"; exit 0; fi

tag=asp_deps_mac_x64_v4
bbUrl=https://github.com/NeoGeographyToolkit/BinaryBuilder/releases/download/${tag}
envParent="$HOME/miniconda3/envs"
wget ${bbUrl}/asp_deps_p1.tar.gz > /dev/null 2>&1
wget ${bbUrl}/python_isis10.tar.gz > /dev/null 2>&1
mkdir -p "$envParent/asp_deps"
cat asp_deps_p*.tar.gz | tar xzf - -C "$envParent/asp_deps"
"$envParent/asp_deps/bin/conda-unpack"
envPath=$(ls -d $HOME/*conda3/envs/asp_deps)
echo "PROBE: envPath=$envPath"
echo "PROBE: default ld:"; ld -v 2>&1 | head -1

CXX="$envPath/bin/clang++"
INC="-I$envPath/include -I$envPath/include/isis -I$envPath/include/qt6 -I$envPath/include/qt6/QtCore"

# FileName.h alone compiles (Cube.h pulls QLinkedList, gone in Qt6).
# FileName::FileName(QString) + FileName::expanded() live in libcore -> the
# representative "undefined isis symbol" ASP hits.
cat > /tmp/isistest.cc <<'CC'
#include <FileName.h>
int dummyfn() { Isis::FileName f("x"); return (int)f.expanded().size(); }
CC

$CXX -std=c++17 $INC -c /tmp/isistest.cc -o /tmp/isistest.o 2>/tmp/cerr
echo "PROBE compile rc=$?"; [ -f /tmp/isistest.o ] || { echo "PROBE: compile FAILED:"; head -6 /tmp/cerr; exit 0; }
echo "PROBE: isistest.o undefined isis syms:"; nm -u /tmp/isistest.o 2>/dev/null | grep -iE 'FileName' | head -3

CONDA_LD=$(ls $envPath/bin/x86_64-apple-darwin*-ld 2>/dev/null | head -1)
echo "PROBE: conda ld = $CONDA_LD ($($CONDA_LD -v 2>&1 | head -1))"

link_test () { # $1=desc  rest=extra clang/link args
  local desc="$1"; shift
  $CXX /tmp/isistest.o -L$envPath/lib -lisis $envPath/lib/libcore.dylib $envPath/lib/libQt6Core.dylib "$@" -Wl,-undefined,error -o /tmp/t_$desc 2>/tmp/e_$desc
  local rc=$?
  echo "PROBE link[$desc] rc=$rc"
  [ $rc -ne 0 ] && grep -iE 'Undefined symbols|symbol.* not found|unknown|unsupported' /tmp/e_$desc | head -2
}

# explicit libcore+libisis, default (new) ld:
link_test "default_ld"
# force classic ld:
link_test "ld_classic" -Wl,-ld_classic
# force conda classic ld64 via -fuse-ld:
[ -n "$CONDA_LD" ] && link_test "fuse_conda_ld" -fuse-ld="$CONDA_LD"

echo "PROBE: install + try lld"
conda install -y -c conda-forge lld > /dev/null 2>&1
LLD=$(ls $envPath/bin/ld64.lld $envPath/bin/lld 2>/dev/null | head -1)
echo "PROBE: lld = ${LLD:-none}"
[ -n "$LLD" ] && link_test "lld" -fuse-ld="$LLD"

echo "PROBE DONE"
exit 0
