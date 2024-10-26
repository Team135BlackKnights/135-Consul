#!/bin/bash

echo "SCPing OrangePi code to OrangePi..."

# Using SCP to copy the file (replace '/path/to/your/file' with the actual file path)
scp /home/lvuser/deploy/OrangePi/ServerSide.py pi@photonvision.local:PyDriverStation/OrangePi/ServerSide.py

# Remotely connecting via SSH to reload the daemon, restart the service, and get its status
STATUS=$(ssh -T pi@photonvision.local << EOF
	echo "Reloading ai-server.service..."
    echo "raspberry" | sudo -S systemctl daemon-reload
    echo "Restarting ai-server.service..."
    echo "raspberry" | sudo -S systemctl restart ai-server.service
    echo "Checking status of ai-server.service..."
    STATUS=\$(sudo -S systemctl is-active ai-server.service)
    echo \$STATUS
EOF
)

# Echoing the service status back to the main script
echo "custom-ai-server.service status: $STATUS"