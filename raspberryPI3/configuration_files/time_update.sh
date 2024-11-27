#!/bin/sh
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
date -s "$(curl http://s3.amazonaws.com -v 2>&1 | \
	grep "Date: " | awk '{ print $3 " " $5 " " $4 " " $7 " " $6 " GMT"}')"
