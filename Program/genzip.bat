if exist robot.zip (
	echo "Deleting robot.zip"
    del robot.zip
) else (
    echo "robot.zip not found"
)

echo "Zipping code into robot.zip"
tar.exe -a -c -f robot.zip *.py
echo "Complete"
