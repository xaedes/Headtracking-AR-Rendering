#!/bin/bash 

newnamespacename=$1
oldnamespacename=${2:-mainnamespace}

if [ -z "$newnamespacename" ]; then
    echo "No new namespace name specified. Do nothing. Exit."
    echo "Usage: $0 newnamespacename [oldnamespacename='mainnamespace']"
    exit 
fi

find . -type f -name "*.txt" -print0 | xargs -0 sed -i "s/$oldnamespacename/$newnamespacename/g"
find . -type f -name "*.cpp" -print0 | xargs -0 sed -i "s/$oldnamespacename/$newnamespacename/g"
find . -type f -name "*.h"   -print0 | xargs -0 sed -i "s/$oldnamespacename/$newnamespacename/g"
find . -type f -name "*.h.*"   -print0 | xargs -0 sed -i "s/$oldnamespacename/$newnamespacename/g"

