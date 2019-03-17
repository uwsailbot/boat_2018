#!/bin/sh

SCRIPT=`realpath "$0"`
SCRIPTPATH=`dirname "$SCRIPT"`

ln -sf ${SCRIPTPATH}/* ~/Arduino/libraries
