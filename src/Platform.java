

import com.bulletphysics.dynamics.RigidBody;

public class Platform {
	float xPos = 0;
	float yPos = 0;
	float zPos = 0;
	float sizeX = 0;
	float sizeY = 0;
	float sizeZ = 0;
	float R = 0;
	float G = 0;
	float B = 0;
	float type = 0;
	RigidBody rigidBody;
	float xRot = 0;
	float yRot = 0;
	float zRot = 0;
	javax.vecmath.Vector3f originalPosition = new javax.vecmath.Vector3f();
	boolean movePlatform = false;
	javax.vecmath.Vector3f newPosition = new javax.vecmath.Vector3f();
	boolean OnPlatform = false;
}
