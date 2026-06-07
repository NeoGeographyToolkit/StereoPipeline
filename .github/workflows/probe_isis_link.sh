#!/bin/bash
# Tiny probe (Mac Intel): does the strict macos-15 runner ld resolve isis
# symbols from EXPLICITLY-linked isis libs (clean fix = link them, no
# dynamic_lookup), or does it need a transitive/lenient linker (e.g. lld)?
# Builds NOTHING big - downloads deps, compiles a 2-class isis test, link-tests.
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

CXX="$envPath/bin/clang++"
INC="-I$envPath/include -I$envPath/include/isis -I$envPath/include/qt6 -I$envPath/include/qt6/QtCore"

cat > /tmp/isistest.cc <<'CC'
#include <FileName.h>
#include <Cube.h>
int dummyfn() {
  Isis::FileName f("x");
  Isis::Cube c;
  return (int)f.expanded().size();
}
CC

echo "PROBE: ld version on this runner:"; ld -v 2>&1 | head -1
echo "PROBE: compile test.cc:"; $CXX -std=c++17 $INC -c /tmp/isistest.cc -o /tmp/isistest.o 2>&1 | head -8; echo "PROBE compile rc=$?"
[ -f /tmp/isistest.o ] || { echo "PROBE: compile failed, abort"; exit 0; }

# isis lib set from the conda manifest (full set, real dylibs)
ISISLIBS=$(python3 -c "import json,glob
m=glob.glob('$envPath/conda-meta/isis-*.json')[0]
d=json.load(open(m))
libs=[f for f in d['files'] if f.startswith('lib/') and f.endswith('.dylib')]
print(' '.join('$envPath/'+f for f in libs))")
echo "PROBE: isis dylib count = $(echo $ISISLIBS | wc -w)"

link_test () { # $1=desc  $2...=libs
  local desc="$1"; shift
  $CXX /tmp/isistest.o -L$envPath/lib "$@" -Wl,-undefined,error -o /tmp/t_$desc 2>/tmp/err_$desc
  local rc=$?
  echo "PROBE link[$desc] rc=$rc"; [ $rc -ne 0 ] && grep -iE 'Undefined symbols|symbol.* not found' /tmp/err_$desc | head -2
}

link_test "libcore"      $envPath/lib/libcore.dylib
link_test "core_isis"    $envPath/lib/libcore.dylib $envPath/lib/libisis.dylib
link_test "ALL_ISIS"     $ISISLIBS
link_test "lisis_only"   -lisis

echo "PROBE: try lld (transitive/lenient?)"
LLD="$envPath/bin/ld64.lld"
if [ ! -x "$LLD" ]; then conda install -y -c conda-forge lld > /dev/null 2>&1; LLD=$(ls $envPath/bin/ld64.lld $envPath/bin/lld 2>/dev/null | head -1); fi
echo "PROBE: lld at: ${LLD:-none}"
if [ -n "$LLD" ]; then
  $CXX /tmp/isistest.o -L$envPath/lib -lisis -fuse-ld="$LLD" -o /tmp/t_lld 2>/tmp/err_lld; echo "PROBE link[lld+lisis] rc=$?"; grep -iE 'Undefined|not found' /tmp/err_lld | head -2
fi
echo "PROBE DONE"
exit 0
