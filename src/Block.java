

import com.bulletphysics.dynamics.RigidBody;

public class Block {
	float sizeX = 0;
	float sizeY = 0;
	float sizeZ = 0;
	float R = 0;
	float G = 0;
	float B = 0;
	RigidBody rigidBody;
	float xRot = 0;
	float yRot = 0;
	float zRot = 0;
	javax.vecmath.Vector3f lastVel = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f lastDot = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f pos = new javax.vecmath.Vector3f();
	javax.vecmath.Quat4f rotation = new javax.vecmath.Quat4f();
	boolean touching = false;
	boolean breakable = false;
}
