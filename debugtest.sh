echo "--- contents of .venv/Scripts ---"
ls .venv/Scripts/ 2>&1 | head -20
echo "--- direct path test ---"
.venv/Scripts/python.exe --version
.venv/Scripts/python.exe -c "import socket; socket.create_connection(('127.0.0.1', 1883), 2).close(); print('CONNECTED')"
echo "exit code: $?"
