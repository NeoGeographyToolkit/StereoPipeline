#! /bin/tcsh -f

#David Shean
#dshean@gmail.com
#3/20/13

#Build script for NASA VisionWorkbench and Ames Stereo Pipeline on OS X
#Assumes all necessary dependencies are installed with homebrew

set brew_root = /usr/local
set src_root = /Users/dshean
set np = 8

#setenv CC $brew_root/bin/gcc-4.7
#setenv CPP $brew_root/bin/cpp-4.7
#setenv CC $brew_root/bin/g++-4.7

#setenv CPPFLAGS "-I${brew_root}/include"
#setenv LDFLAGS "-L${brew_root}/lib"

echo
echo "Building VW"
echo

#Vision Workbench
set vwsrcdir = $src_root/src/visionworkbench
set vwdstdir_root = $src_root/sw/vw

cd $vwsrcdir

#git reset --hard HEAD
#git clean -f
#git pull
#git fetch --all
#git rebase upstream/master

#Check status

set commit_id = `git log upstream/master -n 1 --pretty=format:%H | cut -c 1-7`
set vwdstdir = "${vwdstdir_root}/vw_${commit_id}"

if ($1 == "asponly") then
    goto asp
endif

#Note, the --without-tiff will force vw to use the GDAL internal libtiff/libgeotiff
set config_opt = (--prefix=$vwdstdir --enable-debug)

echo
./autogen

if ($status) goto failed

echo
./configure $config_opt

if ($status) goto failed

echo
make clean
echo
make -j $np 

if ($status) goto failed

echo
make install

if ($status) goto failed

cd $vwdstdir_root
rm latest
ln -s $vwdstdir:t latest

#Ames Stereo Pipeline
asp:

echo
echo "Building ASP"
echo

set aspsrcdir = $src_root/src/StereoPipeline
set aspdstdir_root = $src_root/sw/asp

#NOTE: laslib is available through brew
#set laslib = $src_root/sw/liblas
set laszip = $src_root/sw/laszip

#setenv CPPFLAGS "-I${brew_root}/include -I${vwdstdir}/include"
#setenv CPPFLAGS "-I${vwdstdir}/include -I${laslib}/include -I${laszip}/include"
setenv CPPFLAGS "-I${vwdstdir}/include -I${laszip}/include"
#setenv LDFLAGS "-L${brew_root}/lib -L${vwdstdir}/lib"
#setenv LDFLAGS "-L${vwdstdir}/lib -L${laslib}/lib -L${laszip}/lib"
setenv LDFLAGS "-L${vwdstdir}/lib -L${laszip}/lib"

cd $aspsrcdir

#git reset --hard HEAD
#git clean -f

#git fetch --all
#git rebase upstream/master

#Check status

set commit_id = `git log upstream/master -n 1 --pretty=format:%H | cut -c 1-7`
set aspdstdir = "${aspdstdir_root}/asp_${commit_id}"

set config_opt = (--prefix=$aspdstdir --enable-debug)
set config_opt = ($config_opt --enable-app-point2las --with-liblas --with-laszip)
set config_opt = ($config_opt --enable-app-dem_geoid)

echo
./autogen

if ($status) goto failed

echo
./configure $config_opt

if ($status) goto failed

echo
make clean
echo
make -j $np 

if ($status) goto failed

echo
make install

if ($status) goto failed

cd $aspdstdir_root
rm latest
ln -s $aspdstdir:t latest

exit

failed:

echo "Failed"
exit 1
