#!/bin/bash



#preparation of the enviroment
cd ../..
source devel/setup.bash
d=1
c=0
term=5
a=$(catkin_make | grep 100% )
test -z "$a" && b=0 || b=1

while [ "$b" -lt "$d" ] 
do 
	a=$(catkin_make | grep 100% )
	test -z "$a" && b=0 || b=1
	if [ "$c" -eq "$term" ] 
	then
		break
	fi
	c=$(($c+$d))
done
if [ $c -eq $term ]
then
	catkin_make
fi 

#installation
path2=$(find . -name SofAr-project -print 2>/dev/null | grep 'src/SofAr-project')
cd $path2
#activation of all sh scripts

a=$(find "$(pwd)" -name '*.sh' )
chmod +x $a
echo "All .sh files activated!"
#activation of all py scripts
a=$(find "$(pwd)" -name  *.py ) 
chmod +x $a
echo "All .py files activated!"

