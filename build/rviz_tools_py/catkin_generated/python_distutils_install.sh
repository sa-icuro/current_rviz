#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/centaur/Desktop/ubi_tools/src/rviz_tools_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/centaur/Desktop/ubi_tools/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/centaur/Desktop/ubi_tools/install/lib/python2.7/dist-packages:/home/centaur/Desktop/ubi_tools/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/centaur/Desktop/ubi_tools/build" \
    "/usr/bin/python" \
    "/home/centaur/Desktop/ubi_tools/src/rviz_tools_py/setup.py" \
    build --build-base "/home/centaur/Desktop/ubi_tools/build/rviz_tools_py" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/centaur/Desktop/ubi_tools/install" --install-scripts="/home/centaur/Desktop/ubi_tools/install/bin"
