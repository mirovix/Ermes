#!/bin/bash

src=/home/pi/pacman/results
dst=/media/pi/USB_CUBE1/backup_flight_resullts/

filediff=$(diff -r $src $dst | grep $src | awk '{print  $4}')

echo "logging info..."
diff -r ${src} ${dst} | grep first | awk '{print  $0}'
#diff -r ${src} ${dst} | grep first | awk '{print  $3}'


for i in $filediff
do
	echo "$i"
	cp $src/$i $dst
done

echo "backup completed."

