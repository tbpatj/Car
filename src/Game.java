
import org.lwjgl.BufferUtils;
import org.lwjgl.LWJGLException;
import org.lwjgl.input.Controller;
import org.lwjgl.input.Controllers;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GLContext;
import org.lwjgl.util.glu.Cylinder;
import org.lwjgl.util.glu.Sphere;
import org.lwjgl.util.vector.Matrix4f;
import org.lwjgl.util.vector.Quaternion;
import org.lwjgl.util.vector.Vector3f;
import org.lwjgl.util.vector.Vector4f;

import Utility.SliderRunner;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.CylinderShapeZ;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.opengl.ARBShadowAmbient.GL_TEXTURE_COMPARE_FAIL_VALUE_ARB;
import static org.lwjgl.opengl.EXTFramebufferObject.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL12.GL_CLAMP_TO_EDGE;
import static org.lwjgl.opengl.GL14.*;
import static org.lwjgl.util.glu.GLU.gluLookAt;
import static org.lwjgl.util.glu.GLU.gluPerspective;

/**
 * Shows how to get shadows working in OpenGL. Ported from the OpenGLSuperBible. Some code was modified by Oskar
 * Veerhoek.
 *
 * @author Sam K.
 * @author Daniel W.
 */
public class Game {

    // This represents if the clients computer has the ambient shadow extention
    private static boolean ambientShadowsAvailable;
    // Enable this if you want to see the depth texture for debugging purposes.
    private static boolean showShadowMap = false;
    // Disable this if your computer doesn't support the FBO extension
    private static final boolean useFBO = true;

    // The amount of polygon offset to use
    private static float factor = 4.0F;

    private static int maxTextureSize;

    private static int shadowWidth = 2000;
    private static int shadowHeight = 1800;

    private static int frameBuffer;
    private static int renderBuffer;

    private static final FloatBuffer ambientLight = BufferUtils.createFloatBuffer(4);
    private static final FloatBuffer diffuseLight = BufferUtils.createFloatBuffer(4);
    private static final FloatBuffer lightPosition = BufferUtils.createFloatBuffer(4);
    private static final FloatBuffer cameraPosition = BufferUtils.createFloatBuffer(4);
    private static final FloatBuffer tempBuffer = BufferUtils.createFloatBuffer(4);

    private static final Matrix4f textureMatrix = new Matrix4f();
    private static final Sphere sphere = new Sphere();
    
    //Controler Inputs
    int circle = 0;
    int square = 0;
    int triangle = 0;
    int xButton = 0;
    int rightTrigger = 0;
    int leftTrigger = 0;
    int rightClick = 0;
    int leftClick = 0;
    int dPadUp = 0;
    int dPadDown = 0;
    int dPadRight = 0;
    int dPadLeft = 0;
    int L1 = 0;
    int R1 = 0;
    int select = 0;
    int start = 0;
    //Controller axis's
    int axis1 = 0;
    int axis2 = 1;
    int axis3 = 2;
    int axis4 = 3;
    
    
    final Transform DEFAULT_BALL_TRANSFORM = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(0,0,0,1),new javax.vecmath.Vector3f(0,21,4),1.0f));
    int coolDownShift = 0;
	javax.vecmath.Vector3f stayPos = new javax.vecmath.Vector3f();
	short ground = 4;
	int axisBlock = 0;
	int mainBlock = 0;
	int backWheel1 = 0;
	int backWheel2 = 0;
	int axisBlock2 = 0;
	boolean grabbedSwordHilt = false;
	boolean keyQDown = false;
	boolean grabbed = false;
	boolean pickUpSword = false;
	int mouseWait = 0;
	boolean keyShift = false;
	boolean pickSwordDown = false;
	boolean mouseDown = false;
	short objects = 10;
	short nothing = 1;
	short characterCol = 30;
	//short collideOnlyGround = 50;
	CapsuleShape capsuleS;
	RigidBody capsule;
	ConstraintVec capVec = new ConstraintVec();
	short groundObjects = (short) (nothing + objects + characterCol);
	short objectCollide = (short) (objects + ground + characterCol);
	short capsuleCollide = (short) (objects + ground);
	boolean onGround = false;
	boolean deleteConDown = false;
	boolean reConstrainted = true;
	boolean xDown = false;
	boolean escaped = false;
	float runSpeed = 60;
	boolean snake = false;
	float gravityPull = -20;
	boolean placeSphere = false;
	//Shadow Stuff
	
	    
	javax.vecmath.Vector3f Position1 = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f swordPosition = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f swordPosition2 = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f swordPosition3 = new javax.vecmath.Vector3f();
	javax.vecmath.Quat4f swordRotation = new javax.vecmath.Quat4f();
	javax.vecmath.Quat4f swordRotation2 = new javax.vecmath.Quat4f();
	javax.vecmath.Quat4f swordRotation3 = new javax.vecmath.Quat4f();
	boolean PlaceMovingPlatform = false;
	boolean escapeKey = false;
	float FOV = 25;
	float renderDistance = 1000;
	boolean PlaceKeyDown = false;
	boolean PlaceKeyDown2 = false;
	float cameraDirX = 0;
	float cameraDirY = 0;
	float cameraX = 0;
	float cameraY = 20;
	float cameraZ = 0;
	Generic6DofConstraint generic6DofConstraint2;
	Generic6DofConstraint generic6DofConstraint3;
	Generic6DofConstraint capsuleCon;
	Generic6DofConstraint capsuleCon2;
	RigidBody groundRigidBody;
	RigidBody renderBody;
	SliderRunner sliders = new SliderRunner();
	
	Controller controller;//Controller
	
	List<RigidBody> rigidBodies = new ArrayList<RigidBody>();
	List<SphereC> spheres = new ArrayList<SphereC>();
	List<Block> blocks = new ArrayList<Block>();
	List<Block> swords = new ArrayList<Block>();
	List<Block> character = new ArrayList<Block>();
	List<Block> shatters = new ArrayList<Block>();
	List<Cylinders> cylinders = new ArrayList<Cylinders>();
	List<TypedConstraint> constraints = new ArrayList<TypedConstraint>();
	List<TypedConstraint> characterConstraints = new ArrayList<TypedConstraint>();
	DynamicsWorld dynamicsWorld;
	RigidBody cube;
	RigidBody platform;
	List<Platform> Platforms = new ArrayList<Platform>();
	public void getControllers()
	{
		 try {
	 			Controllers.create();
	 		} catch (LWJGLException e) {
	 			// TODO Auto-generated catch block
	 			e.printStackTrace();
	 		}
	    	if(Controllers.getControllerCount() > 0)
	    	{
	    		controller = Controllers.getController(0);
	    	}
	    	System.out.println(controller.getName());
	    	if(controller.getName().split(" ")[0].equalsIgnoreCase("Wiimote"))
	    	{
	    		circle = 15; //A Button
	    	    square = 18; //Y Button
	    	    triangle = 17; // X Button
	    	    xButton = 16; //B Button
	    	    leftTrigger = 21; // Left Trigger
	    	    rightTrigger = 22; // Right Trigger
	    	    L1 = 19; //L1
	    	    R1 = 20; //R1
	    	    leftClick = 23; //Left Click
	    	    rightClick = 24; //Right Click
	    	    dPadUp = 11; //dPadUp
	    	    dPadDown = 12; //dPadDown
	    	    dPadLeft = 13; //dPadLeft
	    	    dPadRight = 14; //dPadRight
	    	    select = 7; //select
	    	    start = 6; //start
	    	    //Controller axis's
	    	    axis1 = 0; //Left X Axis
	    	    axis2 = 1; //Left Y Axis
	    	    axis3 = 2; //Right X Axis
	    	    axis4 = 3; //Right Y Axis
	    	}
	}
	public void createCyl(float x, float y, float z, float baseR, float topR, float height, float R, float G, float B,float mass, float friction, int res1, int res2,short group, short mask)
	{
		javax.vecmath.Vector3f cyl = new javax.vecmath.Vector3f(baseR,topR,height);
		CylinderShapeZ sphere = new CylinderShapeZ(cyl);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(0,0,0,1),new javax.vecmath.Vector3f(x,y,z),1.0f));
    	MotionState cubeMotionState2 = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia2 = new javax.vecmath.Vector3f(0,0,0);
    	sphere.calculateLocalInertia(mass, cubeInertia2);
       	RigidBodyConstructionInfo cubeConstructionInfo2 = new RigidBodyConstructionInfo(mass,cubeMotionState2,sphere,cubeInertia2);
    	cubeConstructionInfo2.restitution = 0.1f;
    	cubeConstructionInfo2.angularDamping = 0f;
    	//cubeConstructionInfo.mass = 0.1f;
    	cubeConstructionInfo2.friction = friction;
    	cubeConstructionInfo2.mass = mass;
    	RigidBody sphereBody = new RigidBody(cubeConstructionInfo2);
    	sphereBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
		Cylinders cly = new Cylinders();
		cly.baseR = baseR;
		cly.topR = topR;
		cly.height = height;
		cly.R = R;
		cly.G = G;
		cly.B = B;
		cly.res1 = res1;
		cly.res2 = res2;
		cly.rigidBody = sphereBody;
		cylinders.add(cly);
		dynamicsWorld.addCollisionObject(sphereBody,group,mask);
	}
	public void controllerInput()
	{
		javax.vecmath.Vector3f pos1 = blocks.get(axisBlock).rigidBody.getWorldTransform(new Transform()).origin;
		javax.vecmath.Vector3f pos2 = blocks.get(axisBlock2).rigidBody.getWorldTransform(new Transform()).origin;
		javax.vecmath.Vector3f vehi = new javax.vecmath.Vector3f(pos1.x - pos2.x, 0, pos1.z - pos2.z);
		vehi.normalize();
		javax.vecmath.Vector3f pos3 = cylinders.get(0).rigidBody.getWorldTransform(new Transform()).origin;
		javax.vecmath.Vector3f pos4 = cylinders.get(3).rigidBody.getWorldTransform(new Transform()).origin;
		javax.vecmath.Vector3f wheels = new javax.vecmath.Vector3f(pos3.x - pos4.x, 0, pos3.z - pos4.z);
		wheels.normalize();
		float vehicleDir = (float)Math.toDegrees(Math.acos(vehi.z));
		float wheelDir = (float)Math.toDegrees(Math.acos(wheels.z));
		if(vehi.x < 0)
		{
			vehicleDir = vehicleDir * -1;
		}
		if(wheels.x < 0)
		{
			wheelDir = wheelDir * -1;
		}
		wheelDir = wheelDir + 90;
		controller.poll();
		/**
			 Block block = blocks.get(axisBlock);
			 javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
			 block.rigidBody.getAngularVelocity(angVel);
			 angVel.y = angVel.y - 4f;
			 block.rigidBody.setAngularVelocity(angVel);
		 
			 Block block = blocks.get(axisBlock);
			 javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
			 block.rigidBody.getAngularVelocity(angVel);
			 angVel.y = angVel.y + 4f;
			 block.rigidBody.setAngularVelocity(angVel);
			 */
		 Block block = blocks.get(axisBlock);
		 javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
		 block.rigidBody.getAngularVelocity(angVel);
		 angVel.y = angVel.y - (controller.getAxisValue(axis1)* 4);
		 block.rigidBody.setAngularVelocity(angVel);
		 
		 if(controller.isButtonPressed(rightTrigger))
		 {
			 
			Cylinders cyl1 = cylinders.get(1);
			Cylinders cyl2 = cylinders.get(2);
			angVel = new javax.vecmath.Vector3f();
			cyl1.rigidBody.getAngularVelocity(angVel);
			//cyl1.rigidBody.getLinearVelocity(angVel);
			javax.vecmath.Vector3f speed = new javax.vecmath.Vector3f();
			cyl1.rigidBody.getLinearVelocity(speed);
			
			float speedL = speed.length() / 6;
			System.out.println(speedL);
			if(speedL > 9.9)
			{
				speedL = 9.9f;
			}
			angVel.x = angVel.x + (float)(Math.sin(Math.toRadians(vehicleDir + 90)) * (10 - speedL));
			angVel.z = angVel.z + (float)(Math.cos(Math.toRadians(vehicleDir + 90)) * (10 - speedL));
			cyl1.rigidBody.setAngularVelocity(angVel);
			angVel = new javax.vecmath.Vector3f();
			cyl2.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x + (float)(Math.sin(Math.toRadians(vehicleDir + 90)) * (10 - speedL));
			angVel.z = angVel.z + (float)(Math.cos(Math.toRadians(vehicleDir + 90)) * (10 - speedL));
			cyl2.rigidBody.setAngularVelocity(angVel);
			
		 }
		 if(controller.isButtonPressed(leftTrigger))
		 {
			 
			Cylinders cyl1 = cylinders.get(1);
			Cylinders cyl2 = cylinders.get(2);
			angVel = new javax.vecmath.Vector3f();
			cyl1.rigidBody.getAngularVelocity(angVel);
			//cyl1.rigidBody.getLinearVelocity(angVel);
			angVel.x = angVel.x + (float)(Math.sin(Math.toRadians(vehicleDir + 90)) * -1);
			angVel.z = angVel.z + (float)(Math.cos(Math.toRadians(vehicleDir + 90)) * -1);
			cyl1.rigidBody.setAngularVelocity(angVel);
			angVel = new javax.vecmath.Vector3f();
			cyl2.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x + (float)(Math.sin(Math.toRadians(vehicleDir + 90)) * -1);
			angVel.z = angVel.z + (float)(Math.cos(Math.toRadians(vehicleDir + 90)) * -1);
			cyl2.rigidBody.setAngularVelocity(angVel);
			
		 }
		 if(controller.isButtonPressed(xButton))
		 {
			 
			Cylinders cyl1 = cylinders.get(1);
			Cylinders cyl2 = cylinders.get(2);
			angVel = new javax.vecmath.Vector3f();
			cyl1.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x * 0.6f;
			angVel.z = angVel.z * 0.6f;
			cyl1.rigidBody.setAngularVelocity(angVel);
			angVel = new javax.vecmath.Vector3f();
			cyl2.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x * 0.6f;
			angVel.z = angVel.z * 0.6f;
			cyl2.rigidBody.setAngularVelocity(angVel);
			
		 }
		
		 
	}
	public void input2()
	{
		
		
		if(Keyboard.isKeyDown(Keyboard.KEY_ESCAPE))
		{
			if(escapeKey == false)
			{
				Mouse.setGrabbed(false);
				grabbed = false;
				if(escaped)
				{
					escaped = false;
				}
				else
				{
					escaped = true;
				}
				escapeKey = true;
				
			}
		}
		else
		{
			escapeKey = false;
		}
		javax.vecmath.Vector3f pos1 = blocks.get(axisBlock).rigidBody.getWorldTransform(new Transform()).origin;
		javax.vecmath.Vector3f pos2 = blocks.get(axisBlock2).rigidBody.getWorldTransform(new Transform()).origin;
		javax.vecmath.Vector3f vehi = new javax.vecmath.Vector3f(pos1.x - pos2.x, 0, pos1.z - pos2.z);
		vehi.normalize();
		javax.vecmath.Vector3f pos3 = cylinders.get(0).rigidBody.getWorldTransform(new Transform()).origin;
		javax.vecmath.Vector3f pos4 = cylinders.get(3).rigidBody.getWorldTransform(new Transform()).origin;
		javax.vecmath.Vector3f wheels = new javax.vecmath.Vector3f(pos3.x - pos4.x, 0, pos3.z - pos4.z);
		wheels.normalize();
		float vehicleDir = (float)Math.toDegrees(Math.acos(vehi.z));
		float wheelDir = (float)Math.toDegrees(Math.acos(wheels.z));
		if(vehi.x < 0)
		{
			vehicleDir = vehicleDir * -1;
		}
		if(wheels.x < 0)
		{
			wheelDir = wheelDir * -1;
		}
		wheelDir = wheelDir + 90;
		 if(Keyboard.isKeyDown(Keyboard.KEY_D))
		 {
			 Block block = blocks.get(axisBlock);
			 javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
			 block.rigidBody.getAngularVelocity(angVel);
			 angVel.y = angVel.y - 4f;
			 block.rigidBody.setAngularVelocity(angVel);
		 }
		 else if(Keyboard.isKeyDown(Keyboard.KEY_A))
		 {
			 Block block = blocks.get(axisBlock);
			 javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
			 block.rigidBody.getAngularVelocity(angVel);
			 angVel.y = angVel.y + 4f;
			 block.rigidBody.setAngularVelocity(angVel);
		 }
		 else
		 {
			 /**
			 Block block = blocks.get(axisBlock);
			 javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
			
			 if((wheelDir - vehicleDir) > 15)
			 {
				 block.rigidBody.getAngularVelocity(angVel);
				 angVel.y = angVel.y - 1f;
				 block.rigidBody.setAngularVelocity(angVel);
			 }
			 else if((wheelDir - vehicleDir) < -15)
			 {
				 block.rigidBody.getAngularVelocity(angVel);
				 angVel.y = angVel.y + 1f;
				 block.rigidBody.setAngularVelocity(angVel);
			 }
			 else if((wheelDir - vehicleDir) > 5 || (wheelDir - vehicleDir) < -5)
			 {
				 angVel.y = angVel.y * 0.7f;
				 block.rigidBody.setAngularVelocity(angVel);
			 }
			 */
		 }
		 
		 if(Keyboard.isKeyDown(Keyboard.KEY_W))
		 {
			 
			Cylinders cyl1 = cylinders.get(1);
			Cylinders cyl2 = cylinders.get(2);
			javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
			cyl1.rigidBody.getAngularVelocity(angVel);
			//cyl1.rigidBody.getLinearVelocity(angVel);
			angVel.x = angVel.x + (float)(Math.sin(Math.toRadians(vehicleDir + 90)) * 1);
			angVel.z = angVel.z + (float)(Math.cos(Math.toRadians(vehicleDir + 90)) * 1);
			cyl1.rigidBody.setAngularVelocity(angVel);
			angVel = new javax.vecmath.Vector3f();
			cyl2.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x + (float)(Math.sin(Math.toRadians(vehicleDir + 90)) * 1);
			angVel.z = angVel.z + (float)(Math.cos(Math.toRadians(vehicleDir + 90)) * 1);
			cyl2.rigidBody.setAngularVelocity(angVel);
			
		 }
		 
		 
		 if(Keyboard.isKeyDown(Keyboard.KEY_F))
			{
				if(runSpeed < 500)
				{
					runSpeed = runSpeed + 5;
				}
			}
			else
			{
				runSpeed = 60;
			}
		 if(Keyboard.isKeyDown(Keyboard.KEY_S))
		 {
			 
			Cylinders cyl1 = cylinders.get(1);
			Cylinders cyl2 = cylinders.get(2);
			javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
			cyl1.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x + (float)(Math.sin(Math.toRadians(vehicleDir + 90)) * -1);
			angVel.z = angVel.z + (float)(Math.cos(Math.toRadians(vehicleDir + 90)) * -1);
			cyl1.rigidBody.setAngularVelocity(angVel);
			angVel = new javax.vecmath.Vector3f();
			cyl2.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x + (float)(Math.sin(Math.toRadians(vehicleDir + 90)) * -1);
			angVel.z = angVel.z + (float)(Math.cos(Math.toRadians(vehicleDir + 90)) * -1);
			cyl2.rigidBody.setAngularVelocity(angVel);
			
		 }
		 if(Keyboard.isKeyDown(Keyboard.KEY_SPACE))
		 {
			 
			Cylinders cyl1 = cylinders.get(1);
			Cylinders cyl2 = cylinders.get(2);
			javax.vecmath.Vector3f angVel = new javax.vecmath.Vector3f();
			cyl1.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x * 0.6f;
			angVel.z = angVel.z * 0.6f;
			cyl1.rigidBody.setAngularVelocity(angVel);
			angVel = new javax.vecmath.Vector3f();
			cyl2.rigidBody.getAngularVelocity(angVel);
			angVel.x = angVel.x * 0.6f;
			angVel.z = angVel.z * 0.6f;
			cyl2.rigidBody.setAngularVelocity(angVel);
			
		 }
		
			 javax.vecmath.Vector3f pos = cube.getWorldTransform(new Transform()).origin;
			 javax.vecmath.Vector3f toCam = new javax.vecmath.Vector3f(pos.x - cameraX, pos.y - cameraY, pos.z - cameraZ);
			 float length = toCam.length();
			 toCam.normalize();
			/**
				 cameraX = cameraX + toCam.x * (length / 10);
				 cameraY = cameraY + toCam.y * (length / 10);
				 cameraZ = cameraZ + toCam.z * (length / 10);
			 */
			 cameraX = pos.x;
			 cameraY = pos.y;
			 cameraZ = pos.z;
			 
			 
			 toCam = new javax.vecmath.Vector3f(cameraX - pos.x,0, cameraX - pos.z);
			 toCam.normalize();
			 float fakecameraDirY = (float)Math.toDegrees(Math.acos(toCam.z));
			 if(toCam.x < 0)
			 {
				 fakecameraDirY = fakecameraDirY * -1;
			 }
			 fakecameraDirY = fakecameraDirY + 180;
			 cameraDirY = vehicleDir + 180;
			 //System.out.println(fakecameraDirY);
			 
			 
			 toCam = new javax.vecmath.Vector3f(pos.x - cameraX,pos.y - cameraY, pos.z - cameraZ);
			 toCam.normalize();
			 
		 
		 
		 
			 
	}
    public void start()
    {
    	getControllers();
    	setUpDisplay();
        setUpBufferValues();
        setUpPhysics();
		 setUpRenderBody();
		// setUpConstraint();
		// addCharacter();
		 addTrees();
        setUpOpenGL();
        Display.setVSyncEnabled(true);
        while (!Display.isCloseRequested()) {
            render();
            input();
            input2();
            controllerInput();
            if(shatters.size() >= 1)
			 {
				 int ammount = shatters.size();
				 if(ammount > 40)
				 {
					 ammount = 40;
				 }
				 ammount = 41 - ammount;
				 ammount = (int)Math.round(ammount / 2);
				 if(ammount < 1)
				 {
					 ammount = 1;
				 }
				 if(Math.round(Math.random() * ammount) == 1)
				 {
					 int random = (int)Math.round(Math.random() * (shatters.size() - 1));
					 Block block = shatters.get(random);
					 dynamicsWorld.removeRigidBody(block.rigidBody);
					 shatters.remove(random);
				 }
			 }
            if(!escaped)
            {
            	mouseControls();
            }
            //generateShadowMap();
            dynamicsWorld.stepSimulation(1.0f/runSpeed);
            checkbreaks();
            Display.update();
            Display.sync(60);
        }
        cleanUp();
        System.exit(0);
    }
    public static void main(String[] args) {
    	
        Game main = new Game();
        main.start();
    }

    /** Sets up a display. */
    public void setUpDisplay() {
        try {
            Display.setDisplayMode(new DisplayMode(1000, 900));
            Display.setTitle("Shadow Mapping Demo");
            Display.create();
            Display.setResizable(true);
        } catch (LWJGLException e) {
            System.err.println("Couldn't set up the display");
            Display.destroy();
            System.exit(1);
        }
        
    }

    /**
     * This is where anything you want rendered into your world should go.
     *
     * @param drawGround whether to draw the ground
     */
    public void renderObjects(boolean drawGround) {
        if (drawGround) {
            glColor3f(0.0F, 0.0F, 0.9F);
            glNormal3f(0.0F, 1.0F, 0.0F);
            glBegin(GL_QUADS);
            glVertex3f(-100.0F, -25.0F, -100.0F);
            glVertex3f(-100.0F, -25.0F, 100.0F);
            glVertex3f(100.0F, -25.0F, 100.0F);
            glVertex3f(100.0F, -25.0F, -100.0F);
            glEnd();
        }

       
        
        
		 
		 for(int i = 0; i < Platforms.size(); i ++)
		 {
			 Platform p = Platforms.get(i);
			 RigidBody r = p.rigidBody;
			
			 if(p.xRot != 0 || p.yRot != 0 || p.zRot != 0)
			 {
				 javax.vecmath.Vector3f velocity = new javax.vecmath.Vector3f();
				 velocity.x = p.xRot;
				 velocity.y = p.yRot;
				 velocity.z = p.zRot;
				 r.setAngularVelocity(velocity);
				 r.activate();
				 r.applyTorque(velocity);
				 javax.vecmath.Vector3f velocity2 = new javax.vecmath.Vector3f(0,0,0);
				 r.setLinearVelocity(velocity2);
				 r.setMassProps(1, new javax.vecmath.Vector3f(0,0,0));
				 Transform transform = new Transform();
				 r.getWorldTransform(transform);
				 javax.vecmath.Vector3f Position = new javax.vecmath.Vector3f(p.xPos,p.yPos,p.zPos);
				 javax.vecmath.Quat4f rotation = new javax.vecmath.Quat4f();
				
				 transform.getRotation(rotation);
				 Transform newTransform = new Transform(new javax.vecmath.Matrix4f(rotation,Position,1.0f));
				 r.setWorldTransform(newTransform);
				 
				 
			 }
			 
			 if(p.movePlatform)
			 {
				 r.setMassProps(1, new javax.vecmath.Vector3f(0,0,0));
				 Transform transform = new Transform();
				 r.getWorldTransform(transform);
				 javax.vecmath.Vector3f Position = new javax.vecmath.Vector3f(p.xPos,p.yPos,p.zPos);
				 javax.vecmath.Quat4f rotation = new javax.vecmath.Quat4f();
				
				 transform.getRotation(rotation);
				 Transform newTransform = new Transform(new javax.vecmath.Matrix4f(rotation,Position,1.0f));
				 r.setWorldTransform(newTransform);
				 
				 javax.vecmath.Vector3f moveTowards = new javax.vecmath.Vector3f(p.originalPosition.x - p.newPosition.x,p.originalPosition.y - p.newPosition.y,p.originalPosition.z - p.newPosition.z);
				 float fullLength = moveTowards.length();
				 moveTowards.normalize();
				 float moveSpeed = 100;
				 if(p.OnPlatform)
				 {
					 javax.vecmath.Vector3f speed = new javax.vecmath.Vector3f(p.xPos - p.originalPosition.x,p.yPos - p.originalPosition.y,p.zPos - p.originalPosition.z);
					 float length = speed.length();
					 speed.normalize();
					 if(length > 5 && length < fullLength - 5)
					 {
						 p.xPos = p.xPos + speed.x * (5 / -moveSpeed);
						 p.yPos = p.yPos + speed.y * (5 / -moveSpeed);
						 p.zPos = p.zPos + speed.z * (5 / -moveSpeed);
						 speed.x = speed.x * (5 / -moveSpeed) * 60;
						 speed.y = speed.y * (5 / -moveSpeed) * 60;
						 speed.z = speed.z * (5 / -moveSpeed) * 60;
					 }
					 else if(length < 5)
					 {
						 p.xPos = p.xPos + speed.x * ((length / -moveSpeed) - 0.01f);
						 p.yPos = p.yPos + speed.y * ((length / -moveSpeed) - 0.01f);
						 p.zPos = p.zPos + speed.z * ((length / -moveSpeed) - 0.01f);
						 speed.x = speed.x * ((length / -moveSpeed) - 0.01f) * 60;
						 speed.y = speed.y * ((length / -moveSpeed) - 0.01f) * 60;
						 speed.z = speed.z * ((length / -moveSpeed) - 0.01f) * 60;
					 }
					 else
					 {
							 p.xPos = p.xPos + speed.x * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f);
							 p.yPos = p.yPos + speed.y * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f);
							 p.zPos = p.zPos + speed.z * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f);
							 speed.x = speed.x * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f) * 60;
							 speed.y = speed.y * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f) * 60;
							 speed.z = speed.z * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f) * 60;
					 }
					
					 if(length < 0.1f)
					{
							p.OnPlatform = false;
					}
					 
					 r.setLinearVelocity(speed);
				 }
				 else
				 {
					javax.vecmath.Vector3f speed = new javax.vecmath.Vector3f(p.xPos - p.newPosition.x,p.yPos - p.newPosition.y,p.zPos - p.newPosition.z);
					float length = speed.length();
					speed.normalize();
					if(length > 5 && length < fullLength - 5)
					 {
						 p.xPos = p.xPos + speed.x * (5 / -moveSpeed);
						 p.yPos = p.yPos + speed.y * (5 / -moveSpeed);
						 p.zPos = p.zPos + speed.z * (5 / -moveSpeed);
						 speed.x = speed.x * (5 / -moveSpeed) * 60;
						 speed.y = speed.y * (5 / -moveSpeed) * 60;
						 speed.z = speed.z * (5 / -moveSpeed) * 60;
					 }
					else if(length < 5)
					{
						 p.xPos = p.xPos + speed.x * ((length / -moveSpeed) - 0.01f);
						 p.yPos = p.yPos + speed.y * ((length / -moveSpeed) - 0.01f);
						 p.zPos = p.zPos + speed.z * ((length / -moveSpeed) - 0.01f);
						 speed.x = speed.x * ((length / -moveSpeed) - 0.01f) * 60;
						 speed.y = speed.y * ((length / -moveSpeed) - 0.01f) * 60;
						 speed.z = speed.z * ((length / -moveSpeed) - 0.01f) * 60;
					}
					else
					{
						 p.xPos = p.xPos + speed.x * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f);
						 p.yPos = p.yPos + speed.y * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f);
						 p.zPos = p.zPos + speed.z * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f);
						 speed.x = speed.x * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f) * 60;
						 speed.y = speed.y * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f) * 60;
						 speed.z = speed.z * ((((fullLength + 0.1f) - length) / -moveSpeed) - 0.01f) * 60;
					}
					if(length < 0.1f)
					{
						p.OnPlatform = true;
					}
					
					r.setLinearVelocity(speed);
				 }
			 }
		 
			 
			 
			
			 glPushMatrix();
	        	
	        	
	        	
	        		float[] matrix = new float[16];
		        	Transform transform = new Transform();
		        	FloatBuffer transformationBuffer = BufferUtils.createFloatBuffer(16);
		        	
		        	MotionState motionState = r.getMotionState();
		        	motionState.getWorldTransform(transform);
		        	transform.getOpenGLMatrix(matrix);
		        	
		        	
		        	transformationBuffer.clear();
		        	transformationBuffer.put(matrix);
		        	transformationBuffer.flip();
		        	
		        		glMultMatrix(transformationBuffer);
		        	
		        	GL11.glColor3f(p.R, p.G, p.B);
		        	GL11.glBegin(GL11.GL_QUADS);
		        		
		        		GL11.glNormal3f(0, 0, 1);
		        		
		        		
		        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
		        		
		        		
		        		
		        		GL11.glNormal3f(0, 0, -1);
		        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
		        		
		        		GL11.glNormal3f(1, 0, 0);
		        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
		        		
		        		GL11.glNormal3f(-1, 0, 0);
		        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
		        		
		        		GL11.glNormal3f(0, -1, 0);
		        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
		        		
		        		GL11.glNormal3f(0, 1, 0);
		        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
		        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
		        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
		        		
		        	GL11.glEnd();
		        	
		      GL11.glPopMatrix();
		 }
		 
		 for(int i = 0; i < blocks.size(); i ++)
		 {
			 Block p = blocks.get(i);
			 RigidBody r = p.rigidBody;
			 /**
			 javax.vecmath.Vector3f position = r.getWorldTransform(new Transform()).origin;
			 javax.vecmath.Vector3f towards = new javax.vecmath.Vector3f(position.x - 0,position.y - -120, position.z - 0);
			 towards.normalize();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(towards.x * -gravityPull, towards.y * -gravityPull, towards.z * -gravityPull);
			 r.setGravity(gravity);
			 */
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
			 r.setGravity(gravity);
			 
			 glPushMatrix();
	        
			 javax.vecmath.Vector3f velocity = new javax.vecmath.Vector3f();
			 r.getAngularVelocity(velocity);
			 if(p.xRot != 0)
			 {
				 velocity.x = p.xRot;
			 }
			 if(p.yRot != 0)
			 {
				 velocity.y = p.yRot;
			 }
			 if(p.zRot != 0)
			 {
				 velocity.z = p.zRot;
			 }
			 r.setAngularVelocity(velocity);
			
			
	        		float[] matrix = new float[16];
		        	Transform transform = new Transform();
		        	FloatBuffer transformationBuffer = BufferUtils.createFloatBuffer(16);
		        	
		        	MotionState motionState = r.getMotionState();
		        	motionState.getWorldTransform(transform);
		        	transform.getOpenGLMatrix(matrix);
		        	
		        	
		        	transformationBuffer.clear();
		        	transformationBuffer.put(matrix);
		        	transformationBuffer.flip();
		        	
		        		glMultMatrix(transformationBuffer);
		        	
		        		GL11.glColor3f(p.R, p.G, p.B);
			        	GL11.glBegin(GL11.GL_QUADS);
			        		
			        		GL11.glNormal3f(0, 0, 1);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		
			        		GL11.glNormal3f(0, 0, -1);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(1, 0, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(-1, 0, 0);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		
			        		GL11.glNormal3f(0, -1, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(0, 1, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		
			        	GL11.glEnd();
		        	
		      GL11.glPopMatrix();
		 }
		 for(int i = 0; i < shatters.size(); i ++)
		 {
			 Block p = shatters.get(i);
			 RigidBody r = p.rigidBody;
			 /**
			 javax.vecmath.Vector3f position = r.getWorldTransform(new Transform()).origin;
			 javax.vecmath.Vector3f towards = new javax.vecmath.Vector3f(position.x - 0,position.y - -120, position.z - 0);
			 towards.normalize();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(towards.x * -gravityPull, towards.y * -gravityPull, towards.z * -gravityPull);
			 r.setGravity(gravity);
			 */
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
			 r.setGravity(gravity);
			
			 glPushMatrix();
	        
			 javax.vecmath.Vector3f velocity = new javax.vecmath.Vector3f();
			 r.getAngularVelocity(velocity);
			 if(p.xRot != 0)
			 {
				 velocity.x = p.xRot;
			 }
			 if(p.yRot != 0)
			 {
				 velocity.y = p.yRot;
			 }
			 if(p.zRot != 0)
			 {
				 velocity.z = p.zRot;
			 }
			 r.setAngularVelocity(velocity);
			
			
	        		float[] matrix = new float[16];
		        	Transform transform = new Transform();
		        	FloatBuffer transformationBuffer = BufferUtils.createFloatBuffer(16);
		        	
		        	MotionState motionState = r.getMotionState();
		        	motionState.getWorldTransform(transform);
		        	transform.getOpenGLMatrix(matrix);
		        	
		        	
		        	transformationBuffer.clear();
		        	transformationBuffer.put(matrix);
		        	transformationBuffer.flip();
		        	
		        		glMultMatrix(transformationBuffer);
		        	
		        		GL11.glColor3f(p.R, p.G, p.B);
			        	GL11.glBegin(GL11.GL_QUADS);
			        		
			        		GL11.glNormal3f(0, 0, 1);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		
			        		GL11.glNormal3f(0, 0, -1);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(1, 0, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(-1, 0, 0);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		
			        		GL11.glNormal3f(0, -1, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(0, 1, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		
			        	GL11.glEnd();
		        	
		      GL11.glPopMatrix();
		 }
		 for(int i = 0; i < swords.size(); i ++)
		 {
			 Block p = swords.get(i);
			 RigidBody r = p.rigidBody;
			 if(p.touching)
			 {
				 javax.vecmath.Vector3f vel = new javax.vecmath.Vector3f();
				 r.setAngularVelocity(vel);
				 r.setLinearVelocity(vel);
				 r.setWorldTransform(new Transform(new javax.vecmath.Matrix4f(p.rotation,p.pos,1.0f)));
			 }
			 /**
			 javax.vecmath.Vector3f position = r.getWorldTransform(new Transform()).origin;
			 javax.vecmath.Vector3f towards = new javax.vecmath.Vector3f(position.x - 0,position.y - -120, position.z - 0);
			 towards.normalize();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(towards.x * -gravityPull, towards.y * -gravityPull, towards.z * -gravityPull);
			 r.setGravity(gravity);
			 */
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
			 r.setGravity(gravity);
			 glPushMatrix();
	        	
			 javax.vecmath.Vector3f velocity = new javax.vecmath.Vector3f();
			 r.getAngularVelocity(velocity);
			 if(p.xRot != 0)
			 {
				 velocity.x = p.xRot;
			 }
			 if(p.yRot != 0)
			 {
				 velocity.y = p.yRot;
			 }
			 if(p.zRot != 0)
			 {
				 velocity.z = p.zRot;
			 }
			 r.setAngularVelocity(velocity);
			
			
	        		float[] matrix = new float[16];
		        	Transform transform = new Transform();
		        	FloatBuffer transformationBuffer = BufferUtils.createFloatBuffer(16);
		        	
		        	MotionState motionState = r.getMotionState();
		        	motionState.getWorldTransform(transform);
		        	transform.getOpenGLMatrix(matrix);
		        	
		        	
		        	transformationBuffer.clear();
		        	transformationBuffer.put(matrix);
		        	transformationBuffer.flip();
		        	
		        		glMultMatrix(transformationBuffer);
		        	
		        		GL11.glColor3f(p.R, p.G, p.B);
			        	GL11.glBegin(GL11.GL_QUADS);
			        		
			        		GL11.glNormal3f(0, 0, 1);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		
			        		GL11.glNormal3f(0, 0, -1);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(1, 0, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(-1, 0, 0);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		
			        		GL11.glNormal3f(0, -1, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(0, 1, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		
			        	GL11.glEnd();
		        	
		      GL11.glPopMatrix();
		 }
		 for(int i = 0; i < character.size(); i ++)
		 {
			 Block p = character.get(i);
			 RigidBody r = p.rigidBody;
			 /**
			 javax.vecmath.Vector3f position = r.getWorldTransform(new Transform()).origin;
			 javax.vecmath.Vector3f towards = new javax.vecmath.Vector3f(position.x - 0,position.y - -120, position.z - 0);
			 towards.normalize();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(towards.x * -gravityPull, towards.y * -gravityPull, towards.z * -gravityPull);
			 r.setGravity(gravity);
			  javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,-20,0);
			 r.setGravity(gravity);
			 */
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
			 r.setGravity(gravity);
			 glPushMatrix();
			
			 javax.vecmath.Vector3f velocity = new javax.vecmath.Vector3f();
			 r.getAngularVelocity(velocity);
			 if(p.xRot != 0)
			 {
				 velocity.x = p.xRot;
			 }
			 if(p.yRot != 0)
			 {
				 velocity.y = p.yRot;
			 }
			 if(p.zRot != 0)
			 {
				 velocity.z = p.zRot;
			 }
			 r.setAngularVelocity(velocity);
			
			
	        		float[] matrix = new float[16];
		        	Transform transform = new Transform();
		        	FloatBuffer transformationBuffer = BufferUtils.createFloatBuffer(16);
		        	
		        	MotionState motionState = r.getMotionState();
		        	motionState.getWorldTransform(transform);
		        	transform.getOpenGLMatrix(matrix);
		        	
		        	
		        	transformationBuffer.clear();
		        	transformationBuffer.put(matrix);
		        	transformationBuffer.flip();
		        	
		        		glMultMatrix(transformationBuffer);
		        	
		        		GL11.glColor3f(p.R, p.G, p.B);
			        	GL11.glBegin(GL11.GL_QUADS);
			        		
			        		GL11.glNormal3f(0, 0, 1);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		
			        		GL11.glNormal3f(0, 0, -1);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(1, 0, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(-1, 0, 0);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		
			        		GL11.glNormal3f(0, -1, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 - p.sizeY, 0 - p.sizeZ);
			        		
			        		GL11.glNormal3f(0, 1, 0);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		GL11.glVertex3f(0 + p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 - p.sizeZ);
			        		GL11.glVertex3f(0 - p.sizeX, 0 + p.sizeY, 0 + p.sizeZ);
			        		
			        	GL11.glEnd();
		        	
		      GL11.glPopMatrix();
		 }
		 for(int i = 0; i < rigidBodies.size(); i ++)
		 {
			 RigidBody r = rigidBodies.get(i);
			 glPushMatrix();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
			 r.setGravity(gravity);
	        	
			 /**
				 javax.vecmath.Vector3f position = r.getWorldTransform(new Transform()).origin;
				 javax.vecmath.Vector3f towards = new javax.vecmath.Vector3f(position.x - 0,position.y - -120, position.z - 0);
				 towards.normalize();
				 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(towards.x * -gravityPull, towards.y * -gravityPull, towards.z * -gravityPull);
				 r.setGravity(gravity);
				 */
				 glPushMatrix();
		        		float[] matrix = new float[16];
			        	Transform transform = new Transform();
			        	FloatBuffer transformationBuffer = BufferUtils.createFloatBuffer(16);
			        	
			        	MotionState motionState = r.getMotionState();
			        	motionState.getWorldTransform(transform);
			        	transform.getOpenGLMatrix(matrix);
			        	
			        	
			        	transformationBuffer.clear();
			        	transformationBuffer.put(matrix);
			        	transformationBuffer.flip();
			        	
			        		glMultMatrix(transformationBuffer);
			        	
			        	float size = 0.5f;
			        	GL11.glColor3f(1, 1, 1);
			        	GL11.glBegin(GL11.GL_QUADS);
			        		
			        		GL11.glNormal3f(0, 0, 1);
			        		GL11.glVertex3f(0 + size, 0 + size, 0 + size);
			        		GL11.glVertex3f(0 - size, 0 + size, 0 + size);
			        		GL11.glVertex3f(0 - size, 0 - size, 0 + size);
			        		GL11.glVertex3f(0 + size, 0 - size, 0 + size);
			        		
			        		GL11.glNormal3f(0, 0, -1);
			        		GL11.glVertex3f(0 + size, 0 + size, 0 - size);
			        		GL11.glVertex3f(0 + size, 0 - size, 0 - size);
			        		GL11.glVertex3f(0 - size, 0 - size, 0 - size);
			        		GL11.glVertex3f(0 - size, 0 + size, 0 - size);
			        		
			        		GL11.glNormal3f(1, 0, 0);
			        		GL11.glVertex3f(0 + size, 0 + size, 0 + size);
			        		GL11.glVertex3f(0 + size, 0 - size, 0 + size);
			        		GL11.glVertex3f(0 + size, 0 - size, 0 - size);
			        		GL11.glVertex3f(0 + size, 0 + size, 0 - size);
			        		
			        		GL11.glNormal3f(-1, 0, 0);
			        		GL11.glVertex3f(0 - size, 0 + size, 0 + size);
			        		GL11.glVertex3f(0 - size, 0 + size, 0 - size);
			        		GL11.glVertex3f(0 - size, 0 - size, 0 - size);
			        		GL11.glVertex3f(0 - size, 0 - size, 0 + size);
			        		
			        		GL11.glNormal3f(0, -1, 0);
			        		GL11.glVertex3f(0 + size, 0 - size, 0 + size);
			        		GL11.glVertex3f(0 - size, 0 - size, 0 + size);
			        		GL11.glVertex3f(0 - size, 0 - size, 0 - size);
			        		GL11.glVertex3f(0 + size, 0 - size, 0 - size);
			        		
			        		GL11.glNormal3f(0, 1, 0);
			        		GL11.glVertex3f(0 + size, 0 + size, 0 + size);
			        		GL11.glVertex3f(0 + size, 0 + size, 0 - size);
			        		GL11.glVertex3f(0 - size, 0 + size, 0 - size);
			        		GL11.glVertex3f(0 - size, 0 + size, 0 + size);
			        		
			        	GL11.glEnd();
			        	
			      GL11.glPopMatrix();
	        
		 }
		 GL11.glCullFace(GL11.GL_BACK);
		 for(int i = 0; i < spheres.size(); i ++)
		 {
			 SphereC sphere = spheres.get(i);
			 RigidBody r = sphere.sphere;
			 //glPushMatrix();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
			 r.setGravity(gravity);
	        	
			 /**
			 javax.vecmath.Vector3f position = r.getWorldTransform(new Transform()).origin;
			 javax.vecmath.Vector3f towards = new javax.vecmath.Vector3f(position.x - 0,position.y - -120, position.z - 0);
			 towards.normalize();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(towards.x * -gravityPull, towards.y * -gravityPull, towards.z * -gravityPull);
			 r.setGravity(gravity);
			 */
			 if(sphere.res != 0)
			 {
			 glPushMatrix();
	        		float[] matrix = new float[16];
		        	Transform transform = new Transform();
		        	FloatBuffer transformationBuffer = BufferUtils.createFloatBuffer(16);
		        	
		        	MotionState motionState = r.getMotionState();
		        	motionState.getWorldTransform(transform);
		        	transform.getOpenGLMatrix(matrix);
		        	
		        	
		        	transformationBuffer.clear();
		        	transformationBuffer.put(matrix);
		        	transformationBuffer.flip();
		        	
		        		glMultMatrix(transformationBuffer);
		        		Sphere drawSphere = new Sphere();
		        		GL11.glColor3f(sphere.R, sphere.G, sphere.B);
		        		drawSphere.draw(sphere.size, sphere.res, sphere.res);
		        		
		        	
		        	
		        	
		        	
		        	
		      GL11.glPopMatrix();
			 }
		 }
		 for(int i = 0; i < cylinders.size(); i ++)
		 {
			 Cylinders c = cylinders.get(i);
			 RigidBody r = c.rigidBody;
			 //glPushMatrix();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
			 r.setGravity(gravity);
	        	
			 /**
			 javax.vecmath.Vector3f position = r.getWorldTransform(new Transform()).origin;
			 javax.vecmath.Vector3f towards = new javax.vecmath.Vector3f(position.x - 0,position.y - -120, position.z - 0);
			 towards.normalize();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(towards.x * -gravityPull, towards.y * -gravityPull, towards.z * -gravityPull);
			 r.setGravity(gravity);
			 */
			 
			 glPushMatrix();
	        		float[] matrix = new float[16];
		        	Transform transform = new Transform();
		        	FloatBuffer transformationBuffer = BufferUtils.createFloatBuffer(16);
		        	
		        	MotionState motionState = r.getMotionState();
		        	motionState.getWorldTransform(transform);
		        	transform.getOpenGLMatrix(matrix);
		        	
		        	
		        	transformationBuffer.clear();
		        	transformationBuffer.put(matrix);
		        	transformationBuffer.flip();
		        	
		        		glMultMatrix(transformationBuffer);
		        		Cylinder cly = new Cylinder();
		        		GL11.glColor3f(c.R, c.G, c.B);
		        		GL11.glTranslatef(0, 0, -c.height);
		        		cly.draw(c.baseR, c.topR, c.height * 2, c.res1, c.res2);
		        		GL11.glNormal3f(0, 0, 1);
		        		GL11.glBegin(GL11.GL_TRIANGLE_FAN);
		        		GL11.glVertex3f(0, 0, c.height * 2);
		        		float dir = 0;
		        		for(int t = 0; t < c.res1 + 1; t ++)
		        		{
		        			dir = dir - (360 / c.res1);
		        			GL11.glVertex3f((float)Math.sin(Math.toRadians(dir)) * c.baseR, (float)Math.cos(Math.toRadians(dir)) * c.baseR, c.height * 2);
		        		}
		        		GL11.glEnd();
		        		GL11.glNormal3f(0, 0, -1);
		        		GL11.glBegin(GL11.GL_TRIANGLE_FAN);
		        		GL11.glVertex3f(0, 0, 0);
		        		 dir = 0;
		        		for(int t = 0; t < c.res1 + 1; t ++)
		        		{
		        			dir = dir + (360 / c.res1);
		        			GL11.glVertex3f((float)Math.sin(Math.toRadians(dir)) * c.baseR, (float)Math.cos(Math.toRadians(dir)) * c.baseR,0 );
		        		}
		        		GL11.glEnd();
		        	
		        	
		        	
		        	
		        	
		      GL11.glPopMatrix();
			 
		 }
		 
		// GL11.glCullFace(GL11.GL_FRONT);
		 
		 
    }
    
    public void deleteAllConstraints()
	{
		for(int i = 0; i < characterConstraints.size(); i ++)
		{
			TypedConstraint cons = characterConstraints.get(i);
			dynamicsWorld.removeConstraint(cons);
		}
		reConstrainted = false;
	}
    public void createNewSword()
	{
		Block block = character.get(19);
		Block block2 = character.get(20);
		Block block3 = character.get(21);
		character.remove(21);
		character.remove(20);
		character.remove(19);
		if(block.touching == true)
		{
			block.pos = block.rigidBody.getWorldTransform(new Transform()).origin;
			block.rigidBody.getOrientation(block.rotation);
			
			block2.pos = block2.rigidBody.getWorldTransform(new Transform()).origin;
			block2.rigidBody.getOrientation(block2.rotation);
			
			block3.pos = block3.rigidBody.getWorldTransform(new Transform()).origin;
			block3.rigidBody.getOrientation(block3.rotation);
		}
		swords.add(block);
		swords.add(block2);
		swords.add(block3);
		Block block8 = character.get(8);
		javax.vecmath.Vector3f pos = block8.rigidBody.getWorldTransform(new Transform()).origin;
		
		addBlockAt(pos.x,pos.y,pos.z,0.07f,0.2f,0.07f,0f,0f,0f,0.3f,0.5f,0,0,0,0,0,0,true,capsuleCollide,objectCollide,false);
		addBlockAt(pos.x,pos.y,pos.z,0.2f,0.09f,0.1f,0f,0f,0f,0.3f,0.5f,0,0,0,0,0,0,true,capsuleCollide,objectCollide,false);
		addBlockAt(pos.x,pos.y,pos.z,0.12f,0.8f,0.05f,0.9f,0.9f,0.9f,0.3f,0.5f,0,0,0,0,0,0,true,capsuleCollide,objectCollide,false);
		
		
		ConstraintVec cons = new ConstraintVec();
		Block block20 = character.get(19);
		Block block21 = character.get(20);
		Block block22 = character.get(21);
		javax.vecmath.Vector3f vel = new javax.vecmath.Vector3f();
		cube.getLinearVelocity(vel);
		block20.rigidBody.setLinearVelocity(vel);
		block21.rigidBody.setLinearVelocity(vel);
		block22.rigidBody.setLinearVelocity(vel);
		
		cons.setOrigins(0,-0.1f,0f,0,0.2f,0);
		cons.setAngles(0,0,0,0,0,0);
		createConstraint(block20.rigidBody,block21.rigidBody,cons,false,true);
		cons.setOrigins(0,-0.1f,0f,0,0.8f,0);
		createConstraint(block21.rigidBody,block22.rigidBody,cons,false,true);
		cons.setOrigins(0,-0.4f,0f,0,0.04f,0);
		
		
	}
	public void ReConstraint()
	{
		ConstraintVec cons = new ConstraintVec();
		cons.setOrigins(0, 0.2f, 0, 0f, -0.3f, 0);
		cons.setAngles(-0.01f,0,-0.01f,0.01f,0,0.01f);
		
		Block block = character.get(0);//hat
		Block block2 = character.get(1);//hat
		Block block3 = character.get(2);//hat
		Block block4 = character.get(3);//hat
		Block block5 = character.get(4);//hat
		Block block6 = character.get(5);//hat
		Block block7 = character.get(6);
		Block block8 = character.get(7);
		Block block9 = character.get(8);//arm
		Block block10 = character.get(9);//arm
		Block block11 = character.get(10);
		Block block12 = character.get(11);
		Block block13 = character.get(12);
		Block block14 = character.get(13);
		Block block15 = character.get(14);
		
		Block block16 = character.get(15);
		Block block17 = character.get(16);
		Block block18 = character.get(17);
		Block block19 = character.get(18);
		
		Block block20 = character.get(19);
		Block block21 = character.get(20);
		Block block22 = character.get(21);
		
		//Block block23 = character.get(22);
		//Block block23 = character.get(22);
		//Block block3 = blocks.get(2);
		createConstraint(cube,block.rigidBody,cons,false,true);//hat
		cons.setAngles(-11,0,-11,11,0,11);
		cons.setOrigins(0, 0.09f, 0, 0, -0.09f, 0);
		createConstraint(block.rigidBody,block2.rigidBody,cons,false,true);//hat
		//cons.setOrigins(0, 0.1f, 0, 0, -0.1f, 0);
		createConstraint(block2.rigidBody,block3.rigidBody,cons,false,true);//hat
		//cons.setOrigins(0, 0.1f, 0, 0, -0.1f, 0);
		createConstraint(block3.rigidBody,block4.rigidBody,cons,false,true);//hat
		createConstraint(block4.rigidBody,block5.rigidBody,cons,false,true);//hat
		createConstraint(block5.rigidBody,block6.rigidBody,cons,false,true);//hat
		cons.setOrigins(-0.5f,0,0,0.1f,0,0);
		createConstraint(cube,block7.rigidBody,cons,false,true);//neck to head
		cons.setOrigins(-0.1f,0,0,0.1f,0,0);
		createConstraint(block7.rigidBody,block8.rigidBody,cons,false,true);//Torso to neck
		cons.setOrigins(0.0f,0f,0.45f,0.35f,0,0);
		cons.setAngles(0,-57f,229f,0,0f,0f);//fasdl;fj
		createConstraint(block8.rigidBody,block9.rigidBody,cons,false,true);//arm
		cons.setOrigins(0f,0f,-0.45f,0.35f,0,0);
		cons.setAngles(0,0f,229f,0,57f,0f);
		createConstraint(block8.rigidBody,block10.rigidBody,cons,false,true);//arm
		cons.setAngles(-11f,0,0f,11f,0,0f);
		cons.setOrigins(-0.1f,0f,0,0.1f,0,0);
		
		createConstraint(block19.rigidBody,block11.rigidBody,cons,false,true);
		cons.setAngles(0f,-22f,-57f,0f,22f,57f);
		cons.setOrigins(-0.1f,0f,0.3f,0.2f,0,0);
		createConstraint(block11.rigidBody,block12.rigidBody,cons,false,true);
		cons.setAngles(0f,0,0f,0f,0,57f);
		cons.setOrigins(-0.15f,0f,0f,0.25f,0,0);
		createConstraint(block12.rigidBody,block13.rigidBody,cons,false,true);
		cons.setAngles(0f,-22f,-57f,0f,22f,57f);
		cons.setOrigins(-0.1f,0f,-0.3f,0.2f,0,0);
		createConstraint(block11.rigidBody,block14.rigidBody,cons,false,true);
		cons.setAngles(0f,0,0f,0f,0,57f);//
		cons.setOrigins(-0.15f,0f,0f,0.25f,0,0);
		createConstraint(block14.rigidBody,block15.rigidBody,cons,false,true);
		
		cons.setOrigins(-0.09f,0f,0f,0.09f,0,0);
		cons.setAngles(-11f,0,-5f,11f,0,5f);
		createConstraint(block8.rigidBody,block16.rigidBody,cons,false,true);
		createConstraint(block16.rigidBody,block17.rigidBody,cons,false,true);
		createConstraint(block17.rigidBody,block18.rigidBody,cons,false,true);
		createConstraint(block18.rigidBody,block19.rigidBody,cons,false,true);
		cons.setOrigins(0,-0.1f,0f,0,0.2f,0);
		cons.setAngles(0,0,0,0,0,0);
		createConstraint(block20.rigidBody,block21.rigidBody,cons,false,true);
		cons.setOrigins(0,-0.1f,0f,0,0.8f,0);
		createConstraint(block21.rigidBody,block22.rigidBody,cons,false,true);
		cons.setOrigins(0,-0.4f,0f,0,0.04f,0);
		//createConstraint(block22.rigidBody,block23.rigidBody,cons);
		//cons.setAngles(1,1,1,1,1,1);
		//cons.setOrigins(-0.5f,0f,0f,0.5f,0f,0);
		//cons.setOrigins(0,0f,0f,0,1f,0);
		capVec.setOrigins(0, 0, 0, 0, 1f, 0);
		capVec.setAngles(1,1,1,1,1,1);
		createConstraint(cube,capsule,capVec,true,true);
		reConstrainted = true;
		
		capVec.setOrigins(0.7f, 0, 0, 0, 0f, 0);
		capVec.setLinears(-0.1f,-0.1f , -0.1f, 0.1f, 0.1f, 0.1f);
		createConstraint(block11.rigidBody,capsule,capVec,true,true);
	}
	public void createCapsule()
	{
		capsuleS = new CapsuleShape(0.8f,2.2f);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(0,0,0,1),new javax.vecmath.Vector3f(0,20,0),1.0f));
    	MotionState cubeMotionState2 = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia2 = new javax.vecmath.Vector3f(0,0,0);
    	//capsuleS.calculateLocalInertia(2.5f, cubeInertia2);
       	RigidBodyConstructionInfo cubeConstructionInfo2 = new RigidBodyConstructionInfo(2.5f,cubeMotionState2,capsuleS,cubeInertia2);
    	cubeConstructionInfo2.restitution = 0.1f;
    	cubeConstructionInfo2.angularDamping = 0f;
    	//cubeConstructionInfo.mass = 0.1f;
    	cubeConstructionInfo2.friction = 0.3f;
    	//cubeConstructionInfo2.mass = 0.3f;
    	capsule = new RigidBody(cubeConstructionInfo2);
    	 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
		
    	capsule.setGravity(gravity);
    	capsule.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	
    	
    	dynamicsWorld.addCollisionObject(capsule, nothing, capsuleCollide);
    	//dynamicsWorld.addRigidBody(sphereBody);
    	
	}
	public void mouseControls()
	{
		float mouseX = Mouse.getX();
		float mouseY = Mouse.getY();
		mouseX = mouseX / Display.getWidth() * 1000;
		mouseY = mouseY / Display.getHeight() * 900;
		cameraDirX = cameraDirX + ((mouseY - 450) / 10);
		cameraDirY = cameraDirY - ((mouseX - 500) / 7);
		if(grabbed == false)
		{
			Mouse.setGrabbed(true);
			
			
			grabbed = true;
		}
		if(mouseWait > 3)
		{
			mouseWait = 0;
			Mouse.setCursorPosition(Math.round((0.5f) * Display.getWidth()), Math.round((0.5f) * Display.getHeight()));
		}
		mouseWait = mouseWait + 1;
		//System.out.println(((float)(500 / 1000)));
	}
	public void checkbreaks()
	{
		for(int i = 0; i < blocks.size(); i ++)
		 {
			 Block p = blocks.get(i);
			 RigidBody r = p.rigidBody;
			 /**
			 javax.vecmath.Vector3f position = r.getWorldTransform(new Transform()).origin;
			 javax.vecmath.Vector3f towards = new javax.vecmath.Vector3f(position.x - 0,position.y - -120, position.z - 0);
			 towards.normalize();
			 javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(towards.x * -gravityPull, towards.y * -gravityPull, towards.z * -gravityPull);
			 r.setGravity(gravity);
			 */
			// javax.vecmath.Vector3f gravity = new javax.vecmath.Vector3f(0,gravityPull,0);
			 //r.setGravity(gravity);
			 if(p.breakable)
			 {
				javax.vecmath.Vector3f vel = new javax.vecmath.Vector3f();
				r.getLinearVelocity(vel);
				javax.vecmath.Vector3f check = new javax.vecmath.Vector3f(vel.x - p.lastVel.x,vel.y - p.lastVel.y, vel.z - p.lastVel.z);
				int ammount = Math.round(((p.sizeX + p.sizeY + p.sizeZ) / 3) * 2);
				if(ammount <= 1)
				{
					ammount = 2;
				}
				
				if(p.lastDot.dot(check) > 0.5f)
				{
					System.out.println(ammount);
					blocks.remove(i);
					javax.vecmath.Vector3f pos = r.getWorldTransform(new Transform()).origin;
					javax.vecmath.Quat4f rotation = new javax.vecmath.Quat4f();
					r.getOrientation(rotation);
					for(int n = 0; n < ammount; n ++)
					{
						for(int j = 0; j < ammount; j ++)
						{
							for(int t = 0; t < ammount; t ++)
							{
								float addX = (((p.sizeX * 2) / ammount) * t) - (p.sizeX);
								float addY = (((p.sizeY * 2) / ammount) * j) - (p.sizeY);
								float addZ = (((p.sizeZ * 2) / ammount) * n) - (p.sizeZ);
								Quaternion q = new Quaternion();
								
								q.set(new Vector4f((float)rotation.x,(float)rotation.y,(float)rotation.z,rotation.w));
								Quaternion quat = new Quaternion(addX,addY,addZ,1);
								quat = Quaternion.mulInverse(Quaternion.mul(q,quat,quat),q,quat);
								addX = quat.x;
								addY = quat.y;
								addZ = quat.z;
								addShatter(pos.x + addX + ((p.sizeX / ammount)),pos.y + addY + ((p.sizeY / ammount)),pos.z + addZ + ((p.sizeZ / ammount)),(float)(p.sizeX / ammount), (p.sizeY / ammount), (p.sizeZ / ammount),p.R,p.G,p.B,0.3f,1,rotation.x,rotation.y,rotation.z,0,0,0,false,objects,objectCollide,false);
								if(shatters.size() >= 1)
								{
									Block block2 = shatters.get(shatters.size() - 1);
									block2.rigidBody.setLinearVelocity(vel);
								}
								
							}
						}
					}
					dynamicsWorld.removeRigidBody(r);
					
					if(i != 0)
					{
						i = i - 1;
					}
				}
				p.lastDot = check;
				p.lastVel = vel;
			 }
			
		 }
	}
	public void setUpConstraint()
	{
		createCapsule();
		addBlockAt(0,20,0,0.4f,0.1f,0.4f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//hat
		addBlockAt(0,19,0,0.37f,0.1f,0.37f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//hat
		addBlockAt(0,18,0,0.35f,0.1f,0.35f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//hat
		addBlockAt(0,17,0,0.3f,0.1f,0.3f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//hat
		addBlockAt(0,17,0,0.25f,0.1f,0.25f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//hat
		addBlockAt(0,17,0,0.18f,0.1f,0.18f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//hat
		
		
		
		addBlockAt(0,20,0,0.15f,0.1f,0.2f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//neck
		
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//torso
		
		addBlockAt(0,20,0,0.5f,0.1f,0.1f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//arm
		addBlockAt(0,20,0,0.5f,0.1f,0.1f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);//arm
		
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		addBlockAt(0,20,0,0.2f,0.1f,0.1f,0.7f,0.7f,0.7f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		addBlockAt(0,20,0,0.3f,0.1f,0.1f,0.5f,0.2f,0f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		
		addBlockAt(0,20,0,0.2f,0.1f,0.1f,0.7f,0.7f,0.7f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		addBlockAt(0,20,0,0.3f,0.1f,0.1f,0.5f,0.2f,0f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0.5f,0.2f,0f,0.2f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		
		addBlockAt(0,20,0,0.07f,0.2f,0.07f,0.5f,0.2f,0f,0.3f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		addBlockAt(0,20,0,0.2f,0.09f,0.1f,0.5f,0.2f,0f,0.3f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		addBlockAt(0,20,0,0.12f,0.8f,0.05f,0.9f,0.9f,0.9f,0.3f,0.5f,0,0,0,0,0,0,true,characterCol,capsuleCollide,false);
		//addBlockAt(0,20,0,0.1f,0.1f,0.1f,0.9f,0.9f,0.9f,0.3f,0.5f,0,0,0,0,0,0,true,nothing,ground);
		//addBlockAt(0,20,0,0.03f,0.04f,0.03f,0.9f,0.9f,0.9f,0.3f,0.5f,0,0,0,0,0,0,true);
		//addBlockAt(0,16,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0)
		//createSphere(0,19,0,0.4f,1,1);
		//createSphere(1,19,0,0.4f,1,1);
		ConstraintVec cons = new ConstraintVec();
		cons.setOrigins(0, 0.2f, 0, 0f, -0.3f, 0);
		cons.setAngles(-0.01f,0,-0.01f,0.01f,0,0.01f);
		
		Block block = character.get(0);//hat
		Block block2 = character.get(1);//hat
		Block block3 = character.get(2);//hat
		Block block4 = character.get(3);//hat
		Block block5 = character.get(4);//hat
		Block block6 = character.get(5);//hat
		Block block7 = character.get(6);
		Block block8 = character.get(7);
		Block block9 = character.get(8);//arm
		Block block10 = character.get(9);//arm
		Block block11 = character.get(10);
		Block block12 = character.get(11);
		Block block13 = character.get(12);
		Block block14 = character.get(13);
		Block block15 = character.get(14);
		
		Block block16 = character.get(15);
		Block block17 = character.get(16);
		Block block18 = character.get(17);
		Block block19 = character.get(18);
		
		Block block20 = character.get(19);
		Block block21 = character.get(20);
		Block block22 = character.get(21);
		
		//Block block23 = character.get(22);
		//Block block23 = character.get(22);
		//Block block3 = blocks.get(2);
		createConstraint(cube,block.rigidBody,cons,false,true);//hat
		cons.setAngles(-11f,0,-11f,11f,0,11f);
		cons.setOrigins(0, 0.09f, 0, 0, -0.09f, 0);
		createConstraint(block.rigidBody,block2.rigidBody,cons,false,true);//hat
		//cons.setOrigins(0, 0.1f, 0, 0, -0.1f, 0);
		createConstraint(block2.rigidBody,block3.rigidBody,cons,false,true);//hat
		//cons.setOrigins(0, 0.1f, 0, 0, -0.1f, 0);
		createConstraint(block3.rigidBody,block4.rigidBody,cons,false,true);//hat
		createConstraint(block4.rigidBody,block5.rigidBody,cons,false,true);//hat
		createConstraint(block5.rigidBody,block6.rigidBody,cons,false,true);//hat
		cons.setOrigins(-0.5f,0,0,0.1f,0,0);
		createConstraint(cube,block7.rigidBody,cons,false,true);//neck to head
		cons.setOrigins(-0.1f,0,0,0.1f,0,0);
		createConstraint(block7.rigidBody,block8.rigidBody,cons,false,true);//Torso to neck
		cons.setOrigins(0.0f,0f,0.45f,0.35f,0,0);
		cons.setAngles(0,-57f,229f,0,0f,0f);
		createConstraint(block8.rigidBody,block9.rigidBody,cons,false,true);//arm
		cons.setOrigins(0f,0f,-0.45f,0.35f,0,0);
		cons.setAngles(0,0f,229f,0,57f,0f);
		createConstraint(block8.rigidBody,block10.rigidBody,cons,false,true);//arm
		cons.setAngles(-11f,0,0f,11f,0,0f);
		cons.setOrigins(-0.1f,0f,0,0.1f,0,0);
		
		createConstraint(block19.rigidBody,block11.rigidBody,cons,false,true);
		cons.setAngles(0f,-22f,-57f,0f,22f,57f);
		cons.setOrigins(-0.1f,0f,0.3f,0.2f,0,0);
		createConstraint(block11.rigidBody,block12.rigidBody,cons,false,true);
		cons.setAngles(0f,0,0f,0f,0,57f);
		cons.setOrigins(-0.15f,0f,0f,0.25f,0,0);
		createConstraint(block12.rigidBody,block13.rigidBody,cons,false,true);
		cons.setAngles(0f,-22f,-57f,0f,22f,57f);
		cons.setOrigins(-0.1f,0f,-0.3f,0.2f,0,0);
		createConstraint(block11.rigidBody,block14.rigidBody,cons,false,true);
		cons.setAngles(0f,0,0f,0f,0,57f);//
		cons.setOrigins(-0.15f,0f,0f,0.25f,0,0);
		createConstraint(block14.rigidBody,block15.rigidBody,cons,false,true);
		
		cons.setOrigins(-0.09f,0f,0f,0.09f,0,0);
		cons.setAngles(-11f,0,-5f,11f,0,5f);
		createConstraint(block8.rigidBody,block16.rigidBody,cons,false,true);
		createConstraint(block16.rigidBody,block17.rigidBody,cons,false,true);
		createConstraint(block17.rigidBody,block18.rigidBody,cons,false,true);
		createConstraint(block18.rigidBody,block19.rigidBody,cons,false,true);
		
		//createConstraint(block22.rigidBody,block23.rigidBody,cons);
		//cons.setAngles(1,1,1,1,1,1);
		//cons.setOrigins(-0.5f,0f,0f,0.5f,0f,0);
		//cons.setOrigins(0,0f,0f,0,1f,0);
		capVec.setOrigins(0, 0, 0, 0, 1f, 0);
		capVec.setAngles(1,1,1,1,1,1);
		createConstraint(cube,capsule,capVec,true,true);
		capVec.setOrigins(0.7f, 0, 0, 0, 0f, 0);
		capVec.setLinears(-0.1f,-0.1f , -0.1f, 0.1f, 0.1f, 0.1f);
		createConstraint(block11.rigidBody,capsule,capVec,true,true);
		
    	

	}
	
	public void addCharacter()
	{
		//createCapsule();
		int length = blocks.size();
		addBlockAt(0,20,0,0.4f,0.1f,0.4f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//hat
		addBlockAt(0,19,0,0.37f,0.1f,0.37f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//hat
		addBlockAt(0,18,0,0.35f,0.1f,0.35f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//hat
		addBlockAt(0,17,0,0.3f,0.1f,0.3f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//hat
		addBlockAt(0,17,0,0.25f,0.1f,0.25f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//hat
		addBlockAt(0,17,0,0.18f,0.1f,0.18f,0f,0.6f,0.1f,0.1f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//hat
		
		
		
		addBlockAt(0,20,0,0.15f,0.1f,0.2f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//neck
		
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//torso
		
		addBlockAt(0,20,0,0.5f,0.1f,0.1f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//arm
		addBlockAt(0,20,0,0.5f,0.1f,0.1f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);//arm
		
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.2f,0.1f,0.1f,0.7f,0.7f,0.7f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.3f,0.1f,0.1f,0.5f,0.2f,0f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		
		addBlockAt(0,20,0,0.2f,0.1f,0.1f,0.7f,0.7f,0.7f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.3f,0.1f,0.1f,0.5f,0.2f,0f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0f,0.6f,0.1f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.1f,0.2f,0.4f,0.5f,0.2f,0f,0.2f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		
		addBlockAt(0,20,0,0.07f,0.2f,0.07f,0.5f,0.2f,0f,0.3f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.2f,0.09f,0.1f,0.5f,0.2f,0f,0.3f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.12f,0.4f,0.05f,0.9f,0.9f,0.9f,0.3f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,20,0,0.5f,0.5f,0.5f,0.9f,0.9f,0.9f,0.3f,0.5f,0,0,0,0,0,0,false,objects,objectCollide,false);
		//addBlockAt(0,20,0,0.1f,0.1f,0.1f,0.9f,0.9f,0.9f,0.3f,0.5f,0,0,0,0,0,0,true,nothing,ground);
		//addBlockAt(0,20,0,0.03f,0.04f,0.03f,0.9f,0.9f,0.9f,0.3f,0.5f,0,0,0,0,0,0,true);
		//addBlockAt(0,16,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0)
		//createSphere(0,19,0,0.4f,1,1);
		//createSphere(1,19,0,0.4f,1,1);
		ConstraintVec cons = new ConstraintVec();
		cons.setOrigins(0, 0.2f, 0, 0f, -0.3f, 0);
		cons.setAngles(-28f,0,-28f,28f,0,28f);
		
		Block block = blocks.get(0 + length);//hat
		Block block2 = blocks.get(1 + length);//hat
		Block block3 = blocks.get(2 + length);//hat
		Block block4 = blocks.get(3 + length);//hat
		Block block5 = blocks.get(4 + length);//hat
		Block block6 = blocks.get(5 + length);//hat
		Block block7 = blocks.get(6 + length);
		Block block8 = blocks.get(7 + length);
		Block block9 = blocks.get(8 + length);//arm
		Block block10 = blocks.get(9 + length);//arm
		Block block11 = blocks.get(10 + length);
		Block block12 = blocks.get(11 + length);
		Block block13 = blocks.get(12 + length);
		Block block14 = blocks.get(13 + length);
		Block block15 = blocks.get(14 + length);
		
		Block block16 = blocks.get(15 + length);
		Block block17 = blocks.get(16 + length);
		Block block18 = blocks.get(17 + length);
		Block block19 = blocks.get(18 + length);
		
		Block block20 = blocks.get(19 + length);
		Block block21 = blocks.get(20 + length);
		Block block22 = blocks.get(21 + length);
		Block block23 = blocks.get(22 + length);
		
		//Block block23 = character.get(22);
		//Block block23 = character.get(22);
		//Block block3 = blocks.get(2);
		createConstraint(block23.rigidBody,block.rigidBody,cons,false,false);//hat
		cons.setAngles(-11f,0,-11f,11f,0,11f);
		cons.setOrigins(0, 0.09f, 0, 0, -0.09f, 0);
		createConstraint(block.rigidBody,block2.rigidBody,cons,false,false);//hat
		//cons.setOrigins(0, 0.1f, 0, 0, -0.1f, 0);
		createConstraint(block2.rigidBody,block3.rigidBody,cons,false,false);//hat
		//cons.setOrigins(0, 0.1f, 0, 0, -0.1f, 0);
		createConstraint(block3.rigidBody,block4.rigidBody,cons,false,false);//hat
		createConstraint(block4.rigidBody,block5.rigidBody,cons,false,false);//hat
		createConstraint(block5.rigidBody,block6.rigidBody,cons,false,false);//hat
		cons.setOrigins(-0.5f,0,0,0.1f,0,0);
		createConstraint(block23.rigidBody,block7.rigidBody,cons,false,false);//neck to head
		cons.setOrigins(-0.1f,0,0,0.1f,0,0);
		createConstraint(block7.rigidBody,block8.rigidBody,cons,false,false);//Torso to neck
		cons.setOrigins(0.0f,0f,0.45f,0.35f,0,0);
		cons.setAngles(0,-57f,221f,0,0f,0f);
		createConstraint(block8.rigidBody,block9.rigidBody,cons,false,false);//arm
		cons.setOrigins(0f,0f,-0.45f,0.35f,0,0);
		cons.setAngles(0,0f,221f,0,57f,0f);
		createConstraint(block8.rigidBody,block10.rigidBody,cons,false,false);//arm
		cons.setAngles(-11f,0,0f,11f,0,0f);
		cons.setOrigins(-0.1f,0f,0,0.1f,0,0);
		
		createConstraint(block19.rigidBody,block11.rigidBody,cons,false,false);
		cons.setAngles(0f,-22f,-5f,0f,22f,5f);
		cons.setOrigins(-0.1f,0f,0.3f,0.2f,0,0);
		createConstraint(block11.rigidBody,block12.rigidBody,cons,false,false);
		cons.setAngles(0f,0,0f,0f,0,5f);
		cons.setOrigins(-0.15f,0f,0f,0.25f,0,0);
		createConstraint(block12.rigidBody,block13.rigidBody,cons,false,false);
		cons.setAngles(0f,-22f,-5f,0f,22f,5f);
		cons.setOrigins(-0.1f,0f,-0.3f,0.2f,0,0);
		createConstraint(block11.rigidBody,block14.rigidBody,cons,false,false);
		cons.setAngles(0f,0,0f,0f,0,5f);//
		cons.setOrigins(-0.15f,0f,0f,0.25f,0,0);
		createConstraint(block14.rigidBody,block15.rigidBody,cons,false,false);
		
		cons.setOrigins(-0.09f,0f,0f,0.09f,0,0);
		cons.setAngles(-11f,0,-5f,11f,0,5f);
		createConstraint(block8.rigidBody,block16.rigidBody,cons,false,false);
		createConstraint(block16.rigidBody,block17.rigidBody,cons,false,false);
		createConstraint(block17.rigidBody,block18.rigidBody,cons,false,false);
		createConstraint(block18.rigidBody,block19.rigidBody,cons,false,false);
		cons.setOrigins(0,-0.1f,0f,0,0.2f,0);
		cons.setAngles(0,0,0,0,0,0);
		createConstraint(block20.rigidBody,block21.rigidBody,cons,false,false);
		cons.setOrigins(0,-0.1f,0f,0,0.4f,0);
		createConstraint(block21.rigidBody,block22.rigidBody,cons,false,false);
		cons.setOrigins(0,-0.4f,0f,0,0.04f,0);
		//createConstraint(block22.rigidBody,block23.rigidBody,cons);
		//cons.setAngles(1,1,1,1,1,1);
		//cons.setOrigins(-0.5f,0f,0f,0.5f,0f,0);
		//cons.setOrigins(0,0f,0f,0,1f,0);
		
		
    	

	}
	public void create()
	{
		int size = blocks.size();
		addBlockAt(0,20,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,19,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,18,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,17,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,16,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,15,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,14,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,13,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,12,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,11,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		addBlockAt(0,10,0,0.4f,0.4f,0.4f,0f,0.6f,0.1f,0.1f,0.1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		//createSphere(0,19,0,0.4f,1,1);
		//createSphere(1,19,0,0.4f,1,1);
		ConstraintVec cons = new ConstraintVec();
		cons.setOrigins(0, 0.3f, 0, 0, -0.3f, 0);
		cons.setAngles(-5,0,-5,5,0,5);
		Block block = blocks.get(0 + size);
		Block block2 = blocks.get(1 + size);
		Block block3 = blocks.get(2 + size);
		Block block4 = blocks.get(3 + size);
		Block block5 = blocks.get(4 + size);
		Block block6 = blocks.get(5 + size);
		Block block7 = blocks.get(6 + size);
		Block block8 = blocks.get(7 + size);
		Block block9 = blocks.get(8 + size);
		Block block10 = blocks.get(9 + size);
		Block block11 = blocks.get(10 + size);
		//Block block3 = blocks.get(2);
		//createConstraint(cube,block.rigidBody,cons);
		createConstraint(block.rigidBody,block2.rigidBody,cons,false,false);
		createConstraint(block2.rigidBody,block3.rigidBody,cons,false,false);
		createConstraint(block3.rigidBody,block4.rigidBody,cons,false,false);
		createConstraint(block4.rigidBody,block5.rigidBody,cons,false,false);
		createConstraint(block5.rigidBody,block6.rigidBody,cons,false,false);
		createConstraint(block6.rigidBody,block7.rigidBody,cons,false,false);
		createConstraint(block7.rigidBody,block8.rigidBody,cons,false,false);
		createConstraint(block8.rigidBody,block9.rigidBody,cons,false,false);
		createConstraint(block9.rigidBody,block10.rigidBody,cons,false,false);
		createConstraint(block10.rigidBody,block11.rigidBody,cons,false,false);
		
	
    	

	}
	public void createConstraint(RigidBody rigidBody1, RigidBody rigidBody2,ConstraintVec cons, boolean value, boolean addToCharacter)
	{
		 final Transform frameInA = new Transform();
         final Transform frameInB = new Transform();

         frameInA.setIdentity();
         frameInB.setIdentity();
         frameInA.origin.x = cons.originA.x;
         frameInA.origin.y = cons.originA.y;
         frameInA.origin.z = cons.originA.z;
         
         frameInB.origin.x = cons.originB.x;
         frameInB.origin.y = cons.originB.y;
         frameInB.origin.z = cons.originB.z;

         Generic6DofConstraint generic6DofConstraint = new Generic6DofConstraint(rigidBody1, rigidBody2, frameInA, frameInB, false);
         
         generic6DofConstraint.setLinearLowerLimit(cons.linearLow);
         generic6DofConstraint.setLinearUpperLimit(cons.linearUp);
         if(cons.angleLow.x == 1 && cons.angleLow.y == 1 && cons.angleLow.z == 1 && cons.angleUp.x == 1 && cons.angleUp.y == 1 && cons.angleUp.z == 1)
         {
        	
         }
         else
         {
	         generic6DofConstraint.setAngularLowerLimit(cons.angleLow);
	         generic6DofConstraint.setAngularUpperLimit(cons.angleUp);
         }
         
         dynamicsWorld.addConstraint(generic6DofConstraint, true);
         if(addToCharacter)
         {
        	characterConstraints.add(generic6DofConstraint);
         }
         else
         {
        	 constraints.add(generic6DofConstraint);
         }
         if(value)
         {
        	 if(rigidBody1.equals(cube))
        	 {
        		 capsuleCon = generic6DofConstraint;
        	 }
        	 else
        	 {
        		 capsuleCon2 = generic6DofConstraint;
        	 }
         }
        
	}
	public void updateConstraint(Generic6DofConstraint con,ConstraintVec cons)
	{
		
		RigidBody rigidBody1 = con.getRigidBodyA();
		RigidBody rigidBody2 = con.getRigidBodyB();
		dynamicsWorld.removeConstraint(con);
		 final Transform frameInA = new Transform();
         final Transform frameInB = new Transform();

         frameInA.setIdentity();
         frameInB.setIdentity();
         frameInA.origin.x = cons.originA.x;
         frameInA.origin.y = cons.originA.y;
         frameInA.origin.z = cons.originA.z;
         
         frameInB.origin.x = cons.originB.x;
         frameInB.origin.y = cons.originB.y;
         frameInB.origin.z = cons.originB.z;

         con = new Generic6DofConstraint(rigidBody1, rigidBody2, frameInA, frameInB, false);
         
         con.setLinearLowerLimit(cons.linearLow);
         con.setLinearUpperLimit(cons.linearUp);
         if(cons.angleLow.x == 1 && cons.angleLow.y == 1 && cons.angleLow.z == 1 && cons.angleUp.x == 1 && cons.angleUp.y == 1 && cons.angleUp.z == 1)
         {
        	
         }
         else
         {
	         con.setAngularLowerLimit(cons.angleLow);
	         con.setAngularUpperLimit(cons.angleUp);
         }
         dynamicsWorld.addConstraint(con);
        
	}
	//public static final int NO_CONTACT_RESPONSE = 0;
	public void setUpPhysics()
    {
		//TODO create cinimatic rigidbody for characters
    	BroadphaseInterface broadphase = new DbvtBroadphase();
    	CollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
    	CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
    	ConstraintSolver solver = new SequentialImpulseConstraintSolver();
    	dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    	dynamicsWorld.setGravity(new javax.vecmath.Vector3f(0,-20,0));
    	
    	CollisionShape groundShape = new StaticPlaneShape(new javax.vecmath.Vector3f(0,1,0),0.25f);
    	javax.vecmath.Vector3f vec = new javax.vecmath.Vector3f();
    	vec.x = 0.5f;
    	vec.z = 0.5f;
    	vec.y = 0.5f;
    	CollisionShape cubeShape = new BoxShape(vec);
    	MotionState groundMotionState = new DefaultMotionState(new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(0,0,0,1),new javax.vecmath.Vector3f(0,-50.25f,0),1.0f)));
    	RigidBodyConstructionInfo groundBodyConstructionInfo = new RigidBodyConstructionInfo(0,groundMotionState,groundShape, new javax.vecmath.Vector3f(0,0,0));
    	groundBodyConstructionInfo.restitution = 0f;
    	groundBodyConstructionInfo.friction = 0.4f;
    	groundBodyConstructionInfo.mass = 0;
    	groundRigidBody = new RigidBody(groundBodyConstructionInfo);
    	dynamicsWorld.addCollisionObject(groundRigidBody,ground,groundObjects);
    	MotionState cubeMotionState = new DefaultMotionState(DEFAULT_BALL_TRANSFORM);
    	javax.vecmath.Vector3f cubeInertia = new javax.vecmath.Vector3f(0,0,0);
    	cubeShape.calculateLocalInertia(2.5f, cubeInertia);
    	RigidBodyConstructionInfo cubeConstructionInfo = new RigidBodyConstructionInfo(2.5f,cubeMotionState,cubeShape,cubeInertia);
    	cubeConstructionInfo.restitution = 0f;
    	cubeConstructionInfo.angularDamping = 0.3f;
    	cubeConstructionInfo.friction = 0f;
    	cubeConstructionInfo.mass = 2.5f;
    	//cubeConstructionInfo.
    	
    	cube = new RigidBody(cubeConstructionInfo);
    	cube.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	CollisionFlags c = new CollisionFlags();
    	//dynamicsWorld.addRigidBody(cube);
    	dynamicsWorld.addCollisionObject(cube,characterCol,capsuleCollide);
    	rigidBodies.add(cube);
    	addCubeGround();
    	addCubes();
    	
    }
	public void createSphere(float x, float y, float z,float size,float friction, float mass,int res)
	{
		SphereShape sphere = new SphereShape(size);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(0,0,0,1),new javax.vecmath.Vector3f(x,y,z),1.0f));
    	MotionState cubeMotionState2 = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia2 = new javax.vecmath.Vector3f(0,0,0);
    	sphere.calculateLocalInertia(mass, cubeInertia2);
       	RigidBodyConstructionInfo cubeConstructionInfo2 = new RigidBodyConstructionInfo(mass,cubeMotionState2,sphere,cubeInertia2);
    	cubeConstructionInfo2.restitution = 0.1f;
    	cubeConstructionInfo2.angularDamping = 0f;
    	//cubeConstructionInfo.mass = 0.1f;
    	cubeConstructionInfo2.friction = friction;
    	cubeConstructionInfo2.mass = mass;
    	RigidBody sphereBody = new RigidBody(cubeConstructionInfo2);
    	sphereBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	
    	SphereC spherec = new SphereC();
    	spherec.R = (float)Math.random() * 0.5f + 0.5f;
    	spherec.G = (float)Math.random() * 0.5f + 0.5f;
    	spherec.B = (float)Math.random() * 0.5f + 0.5f;
    	spherec.sphere = sphereBody;
    	spherec.size = size;
    	spherec.res = res;
    	dynamicsWorld.addCollisionObject(sphereBody, nothing, ground);
    	//dynamicsWorld.addRigidBody(sphereBody);
    	spheres.add(spherec);
    	
	}
	public void addCubeGround()
	{
		javax.vecmath.Vector3f vec = new javax.vecmath.Vector3f();
    	vec.x = 400f;
    	vec.z = 400f;
    	vec.y = 2f;
    	CollisionShape cubeShape = new BoxShape(vec);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(0,0,0,1),new javax.vecmath.Vector3f(0,-1.75f,0),1.0f));
    	
    	MotionState cubeMotionState = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia = new javax.vecmath.Vector3f(0,0,0);
    	cubeShape.calculateLocalInertia(2.5f, cubeInertia);
    	RigidBodyConstructionInfo cubeConstructionInfo = new RigidBodyConstructionInfo(2.5f,cubeMotionState,cubeShape,cubeInertia);
    	cubeConstructionInfo.restitution = 0f;
    	cubeConstructionInfo.angularDamping = 0.95f;
    	//cubeConstructionInfo.mass = 0.1f;
    	cubeConstructionInfo.friction = 1f;
    	cubeConstructionInfo.mass = 0;
    	platform = new RigidBody(cubeConstructionInfo);
    	platform.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	Platform plat = new Platform();
    	plat.xPos = 0;
    	plat.yPos = -0.75f;
    	plat.zPos = 0;
    	plat.sizeX = vec.x;
    	plat.sizeY = vec.y;
    	plat.sizeZ = vec.z;
    	plat.R = 0.5f;
    	plat.G = 1;
    	plat.B = 0.1f;
    	plat.type = 0;
    	plat.rigidBody = platform;
    	Platforms.add(plat);
    	//rigidBodies.add(cubey);
    	
    	dynamicsWorld.addCollisionObject(platform,ground,groundObjects);
    	//addPlatformAt(12,-1,0,2,0.025f,2,0.6f,0.6f,0.6f,0.4f,0,0,0,0,0,0,false);
    	//addPlatformAt(12,-0.5f,2,0.025f,0.5f,2,0.6f,0.6f,0.6f,0.4f,0,0,0,0,0,0,false);
    	//addPlatformAt(12,-0.5f,-2,0.025f,0.5f,2,0.6f,0.6f,0.6f,0.4f,0,0,0,0,0,0,false);
    	//addPlatformAt(14,-0.5f,0,2,0.5f,0.025f,0.6f,0.6f,0.6f,0.4f,0,0,0,0,0,0,false);
    	
	}
	public void addTrees()
	{
		for(int i = 0; i < 100; i ++)
		{
			addTreeAt((float)(Math.random() * 740 - 360), 0,(float)(Math.random() * 740 - 340));
		}
		setUpEnvironment();
	}
	public void setUpEnvironment()
	{
		/**
		addPlatformAt(-21,10, 50, 20, 10, 1, 0.8f,0.8f,0.8f,0.2f,0,0,0,0,0,0,false);//LeftFrontWall
		addPlatformAt(21,10, 50, 20, 10, 1, 0.8f,0.8f,0.8f,0.2f,0,0,0,0,0,0,false);//RightFrontWall
		addPlatformAt(0,11, 50, 1, 9, 1, 0.8f,0.8f,0.8f,0.2f,0,0,0,0,0,0,false);//MiddleFrontwall
		addPlatformAt(0,20, 55, 44, 1, 10, 0.8f,0.8f,0.8f,0.2f,0,0,0,0,0,0,false);//wallRoof
		addPlatformAt(0,-0.55f, 57, 40, 1, 8, 0.8f,0.8f,0.8f,0.2f,0,0,0,0,0,0,false);//WallGround
		addPlatformAt(0,10, 64, 30, 10, 1, 0.8f,0.8f,0.8f,0.2f,0,0,0,0,0,0,false);//WallBackWall
		addBlockAt(0,1.2f,49.5f,1f,0.75f,1f,0.8f,0.8f,0.8f,0.4f,1f,0,0,0,0,0,0,false,objects,objectCollide,false);//WallSlideBlock
		
		addPlatformAt(40,10, 70, 1, 10, 20, 0.8f,0.8f,0.8f,0.2f,0,0,0,0,0,0,false);//leftSideWall
		*/
		for(int i = 0; i < 20; i ++)
		{
			addPlatformAt((float)(Math.random() * 740 - 360),0, (float)(Math.random() * 740 - 340), 12, 0.4f, 12, 0.8f,0.8f,0.8f,0.2f,(float)Math.toRadians(Math.random() * 20 - 10),0,(float)Math.toRadians(Math.random() * 20 - 10),0,0,0,false);
		}
		addBlockAt(10,20,0,4f,0.4f,2f,0.8f,0.8f,0.8f,1f,1f,0,0,0,0,0,0,false,objects,objectCollide,false);
		mainBlock = blocks.size() - 1;
		Block b = blocks.get(blocks.size() - 1);
		addBlockAt(10,20,0,0.2f,0.2f,2f,0.8f,0.8f,0.8f,0.4f,1f,0,0,0,0,0,0,false,nothing,objectCollide,false);
		Block b2 = blocks.get(blocks.size() - 1);
		axisBlock = blocks.size() - 1;
		
		
		
		addBlockAt(10,20,0,0.2f,0.2f,2f,0.8f,0.8f,0.8f,0.4f,1f,0,0,0,0,0,0,false,nothing,objectCollide,false);
		Block b8 = blocks.get(blocks.size() - 1);
		axisBlock2 = blocks.size() - 1;
		for(int i = 0; i < 20; i ++)
		{
			addBlockAt((float)(Math.random() * 160 - 80), 2.1f,(float)(Math.random() * 160 - 80),2f,2,2f,0.8f,0.8f,0.8f,0.4f,1f,0,0,0,0,0,0,false,objects,objectCollide,true);
		}
		
		
		createCyl(10,20,0,1f,1f,0.3f,0.2f,0.2f,0.2f,1,1,20,20,nothing,objectCollide);
		createCyl(10,20,0,1f,1f,0.3f,0.2f,0.2f,0.2f,1,1,20,20,objects,objectCollide);
		createCyl(10,20,0,1f,1f,0.3f,0.2f,0.2f,0.2f,1,1,20,20,objects,objectCollide);
		createCyl(10,20,0,1f,1f,0.3f,0.2f,0.2f,0.2f,1,1,20,20,nothing,objectCollide);
		Cylinders c = cylinders.get(0);
		Cylinders c2 = cylinders.get(1);
		Cylinders c3 = cylinders.get(2);
		Cylinders c4 = cylinders.get(3);
		ConstraintVec con = new ConstraintVec();
		
		con.setLinears(0, 0, 0, 0, 0, 0);
		con.setAngles(0, 0, 24, 0, 0, 0);
		con.setOrigins(0, 0f, 0, -4, 0f, 2.3f);
		createConstraint(c2.rigidBody,b.rigidBody,con,false,false);
		con.setOrigins(0, 0f, 0, -4, 0f, -2.3f);
		createConstraint(c3.rigidBody,b.rigidBody,con,false,false);
		con.setAngles(0, -10, 24, 0, 10, 0);
		con.setOrigins(0, 0, 0, 4, 0, -2.3f);
		//createConstraint(c4.rigidBody,b.rigidBody,con,false,false);
		con.setAngles(0, 0, 24, 0, 0, 0);
		con.setOrigins(0, 0, 0, 0, 0, -2.3f);
		createConstraint(c4.rigidBody,b2.rigidBody,con,false,false);
		con.setAngles(0, 0, 24, 0, 0, 0);
		con.setOrigins(0, 0, 0, 0, 0, 2.3f);
		createConstraint(c.rigidBody,b2.rigidBody,con,false,false);
		con.setAngles(0, -35, 0, 0, 35, 0);
		con.setOrigins(4, 0f, 0, 0f, 0, 0f);
		createConstraint(b.rigidBody,b2.rigidBody,con,false,false);
		con.setAngles(0, 0, 0, 0,0, 0);
		con.setOrigins(3, -1.4f, 0, 0f, 0, 0f);
		//createConstraint(b3.rigidBody,b.rigidBody,con,false,false);
		con.setOrigins(-2.5f, -1.4f, 0, 0f, 0, 0f);
		//createConstraint(b4.rigidBody,b.rigidBody,con,false,false);
		con.setOrigins(0.5f, -1.4f, 1.9f, 0f, 0, 0f);
		//createConstraint(b5.rigidBody,b.rigidBody,con,false,false);
		con.setOrigins(0.5f, -1.4f, -1.9f, 0f, 0, 0f);
		//createConstraint(b6.rigidBody,b.rigidBody,con,false,false);
		con.setOrigins(0f, -0f, 0f, 0f, 0, 0f);
		
		createConstraint(cube,b.rigidBody,con,false,false);
		con.setAngles(0, 0, 0, 0, 0, 0);
		con.setOrigins(-4, 0f, 0, 0f, 0, 0f);
		createConstraint(b.rigidBody,b8.rigidBody,con,false,false);
		
		
	}
	public void addTreeAt(float x, float y, float z)
	{
		float size = (float)(Math.random() * 1f + 0.5f);
		float size2 = (float)(Math.random() * 3 + 3f);
		float size3 = (float)(Math.random() * 4f + 4f);
		addPlatformAt(x,y + size2, z, size, size2, size, 0.5f,0.3f,0,0.2f,0,0,0,0,0,0,false);
		addPlatformAt(x,y + (size2 * 2 + (size3 - 1)), z, size3, size3 - 1, size3, 0f,1f,0,0.2f,0,0,0,0,0,0,false);
		y = (size2 * 2 + (size3 - 1));
		
		
		
	}
	public void addPlatformAt(float x, float y, float z, float sizeX, float sizeY, float sizeZ, float R, float G, float B, float friction,float xRot, float yRot, float zRot, float xSpin, float ySpin, float zSpin, boolean movePositions)
	{
		javax.vecmath.Vector3f vec = new javax.vecmath.Vector3f();
    	vec.x = sizeX;
    	vec.y = sizeY;
    	vec.z = sizeZ;
    	CollisionShape cubeShape = new BoxShape(vec);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(xRot,yRot,zRot,1),new javax.vecmath.Vector3f(x,y,z),1.0f));
    	
    	MotionState cubeMotionState = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia = new javax.vecmath.Vector3f(0,0,0);
    	cubeShape.calculateLocalInertia(2.5f, cubeInertia);
    	RigidBodyConstructionInfo cubeConstructionInfo = new RigidBodyConstructionInfo(2.5f,cubeMotionState,cubeShape,cubeInertia);
    	cubeConstructionInfo.restitution = 0f;
    	cubeConstructionInfo.angularDamping = 0.95f;
    	cubeConstructionInfo.friction = friction;
    	cubeConstructionInfo.mass = 0;
    	RigidBody newPlatform = new RigidBody(cubeConstructionInfo);
    	newPlatform.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	//dynamicsWorld.addRigidBody(newPlatform);
    	dynamicsWorld.addCollisionObject(newPlatform,ground,groundObjects);
    	
    	Platform plat = new Platform();
    	plat.xPos = x;
    	plat.yPos = y;
    	plat.zPos = z;
    	plat.sizeX = sizeX;
    	plat.sizeY = sizeY;
    	plat.sizeZ = sizeZ;
    	plat.R = R;
    	plat.G = G;
    	plat.B = B;
    	plat.type = 0;
    	plat.rigidBody = newPlatform;
    	plat.xRot = xSpin;
    	plat.yRot = ySpin;
    	plat.zRot = zSpin;
    	plat.movePlatform = movePositions;
    	if(movePositions)
    	{
    		plat.newPosition = Position1;
    		plat.originalPosition = new javax.vecmath.Vector3f(x,y,z);
    	}
    	Platforms.add(plat);
    	//rigidBodies.add(cubey);
    	
    	
    	
	}
	public void setUpRenderBody()
	{
		javax.vecmath.Vector3f vec = new javax.vecmath.Vector3f();
    	vec.x = 1;
    	vec.y = 1;
    	vec.z = 1;
    	CollisionShape cubeShape = new BoxShape(vec);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(0,-1,0,1),new javax.vecmath.Vector3f(0,0,0),1.0f));
    	
    	MotionState cubeMotionState = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia = new javax.vecmath.Vector3f(0,0,0);
    	cubeShape.calculateLocalInertia(2.5f, cubeInertia);
    	RigidBodyConstructionInfo cubeConstructionInfo = new RigidBodyConstructionInfo(2.5f,cubeMotionState,cubeShape,cubeInertia);
    	cubeConstructionInfo.restitution = 0f;
    	cubeConstructionInfo.angularDamping = 0f;
    	cubeConstructionInfo.friction = 0;
    	cubeConstructionInfo.mass = 0;
    	renderBody = new RigidBody(cubeConstructionInfo);
    	renderBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	dynamicsWorld.addRigidBody(renderBody);
	}
	public void addBlockAt(float x, float y, float z, float sizeX, float sizeY, float sizeZ, float R, float G, float B, float mass, float friction,float xRot, float yRot,float zRot,float xSpin, float ySpin, float zSpin,boolean addTo,short group, short mask,boolean breakable)
	{
		javax.vecmath.Vector3f vec = new javax.vecmath.Vector3f();
    	vec.x = sizeX;
    	vec.y = sizeY;
    	vec.z = sizeZ;
    	CollisionShape cubeShape = new BoxShape(vec);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(xRot,yRot,zRot,1),new javax.vecmath.Vector3f(x,y,z),1.0f));
    	
    	MotionState cubeMotionState = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia = new javax.vecmath.Vector3f(0,0,0);
    	cubeShape.calculateLocalInertia(mass, cubeInertia);
    	RigidBodyConstructionInfo cubeConstructionInfo = new RigidBodyConstructionInfo(mass,cubeMotionState,cubeShape,cubeInertia);
    	cubeConstructionInfo.restitution = 0f;
    	cubeConstructionInfo.angularDamping = 0f;
    	cubeConstructionInfo.friction = friction;
    	cubeConstructionInfo.mass = mass;
    	RigidBody newPlatform = new RigidBody(cubeConstructionInfo);
    	newPlatform.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	dynamicsWorld.addCollisionObject(newPlatform,group,mask);
    	
    	Block block = new Block();
    	block.sizeX = sizeX;
    	block.sizeY = sizeY;
    	block.sizeZ = sizeZ;
    	
    	block.R = R;
    	block.G = G;
    	block.B = B;
    	block.xRot = xSpin;
    	block.yRot = ySpin;
    	block.zRot = zSpin;
    	block.rigidBody = newPlatform;
    	block.breakable = breakable;
    	if(addTo == false)
    	{
    		blocks.add(block);
    	}
    	else
    	{
    		character.add(block);
    	}
    	//rigidBodies.add(cubey);
    	
    	
    	
	}
	public void addShatter(float x, float y, float z, float sizeX, float sizeY, float sizeZ, float R, float G, float B, float mass, float friction,float xRot, float yRot,float zRot,float xSpin, float ySpin, float zSpin,boolean addTo,short group, short mask,boolean breakable)
	{
		javax.vecmath.Vector3f vec = new javax.vecmath.Vector3f();
    	vec.x = sizeX;
    	vec.y = sizeY;
    	vec.z = sizeZ;
    	CollisionShape cubeShape = new BoxShape(vec);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(xRot,yRot,zRot,1),new javax.vecmath.Vector3f(x,y,z),1.0f));
    	
    	MotionState cubeMotionState = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia = new javax.vecmath.Vector3f(0,0,0);
    	cubeShape.calculateLocalInertia(mass, cubeInertia);
    	RigidBodyConstructionInfo cubeConstructionInfo = new RigidBodyConstructionInfo(mass,cubeMotionState,cubeShape,cubeInertia);
    	cubeConstructionInfo.restitution = 0f;
    	cubeConstructionInfo.angularDamping = 0f;
    	cubeConstructionInfo.friction = friction;
    	cubeConstructionInfo.mass = mass;
    	RigidBody newPlatform = new RigidBody(cubeConstructionInfo);
    	newPlatform.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	dynamicsWorld.addCollisionObject(newPlatform,group,mask);
    	
    	Block block = new Block();
    	block.sizeX = sizeX;
    	block.sizeY = sizeY;
    	block.sizeZ = sizeZ;
    	
    	block.R = R;
    	block.G = G;
    	block.B = B;
    	block.xRot = xSpin;
    	block.yRot = ySpin;
    	block.zRot = zSpin;
    	block.rigidBody = newPlatform;
    	block.breakable = breakable;
    	if(addTo == false)
    	{
    		shatters.add(block);
    	}
    	else
    	{
    		//character.add(block);
    	}
    	//rigidBodies.add(cubey);
    	
    	
    	
	}
	public void addCubes()
	{
		javax.vecmath.Vector3f vec = new javax.vecmath.Vector3f();
    	vec.x = 0.5f;
    	vec.z = 0.5f;
    	vec.y = 0.5f;
    	CollisionShape cubeShape = new BoxShape(vec);
    	final Transform cubeTransform = new Transform(new javax.vecmath.Matrix4f(new javax.vecmath.Quat4f(0,-1,0,1),new javax.vecmath.Vector3f(0,6.5f,0),1.0f));
    	
    	MotionState cubeMotionState = new DefaultMotionState(cubeTransform);
    	javax.vecmath.Vector3f cubeInertia = new javax.vecmath.Vector3f(0,0,0);
    	cubeShape.calculateLocalInertia(2.5f, cubeInertia);
    	RigidBodyConstructionInfo cubeConstructionInfo = new RigidBodyConstructionInfo(2.5f,cubeMotionState,cubeShape,cubeInertia);
    	cubeConstructionInfo.restitution = 0.9f;
    	cubeConstructionInfo.angularDamping = 0.95f;
    	//cubeConstructionInfo.mass = 0.1f;
    	cubeConstructionInfo.friction = 1;
    	RigidBody cubey = new RigidBody(cubeConstructionInfo);
    	cubey.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    	
    	rigidBodies.add(cubey);
    	
    	dynamicsWorld.addRigidBody(cubey);
	}
    /** Generate the shadow map. */
   public void generateShadowMap() {
        float lightToSceneDistance, nearPlane, fieldOfView;
        FloatBuffer lightModelView = BufferUtils.createFloatBuffer(16);
        FloatBuffer lightProjection = BufferUtils.createFloatBuffer(16);
        Matrix4f lightProjectionTemp = new Matrix4f();
        Matrix4f lightModelViewTemp = new Matrix4f();

        float sceneBoundingRadius = 95.0F;

        lightToSceneDistance = (float) Math.sqrt(lightPosition.get(0) * lightPosition.get(0) + lightPosition.get(1) *
                lightPosition.get(1) + lightPosition.get(2) * lightPosition.get(2));

        nearPlane = lightToSceneDistance - sceneBoundingRadius;

        fieldOfView = (float) Math.toDegrees(2.0F * Math.atan(sceneBoundingRadius / lightToSceneDistance));

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fieldOfView, 1.0F, nearPlane, nearPlane + (2.0F * sceneBoundingRadius));
        glGetFloat(GL_PROJECTION_MATRIX, lightProjection);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(lightPosition.get(0), lightPosition.get(1), lightPosition.get(2), 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F);
        glGetFloat(GL_MODELVIEW_MATRIX, lightModelView);
        glViewport(0, 0, shadowWidth, shadowHeight);
        
        if (useFBO) {
            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frameBuffer);
        }

        glClear(GL_DEPTH_BUFFER_BIT);

        // Set rendering states to the minimum required, for speed.
        glShadeModel(GL_FLAT);
        glDisable(GL_LIGHTING);
        glDisable(GL_COLOR_MATERIAL);
        glDisable(GL_NORMALIZE);
        glColorMask(false, false, false, false);

        glEnable(GL_POLYGON_OFFSET_FILL);
        GL11.glCullFace(GL11.GL_BACK);
        renderObjects(false);

        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, 0, 0, shadowWidth, shadowHeight, 0);

        // Unbind the framebuffer if we are using them.
        if (useFBO) {
            glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        }

        // Setup the rendering states.
        glShadeModel(GL_SMOOTH);
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glEnable(GL_NORMALIZE);
        glColorMask(true, true, true, true);
        glDisable(GL_POLYGON_OFFSET_FILL);

        lightProjectionTemp.load(lightProjection);
        lightModelViewTemp.load(lightModelView);
        lightProjection.flip();
        lightModelView.flip();

        Matrix4f tempMatrix = new Matrix4f();
        tempMatrix.setIdentity();
        tempMatrix.translate(new Vector3f(0.5F, 0.5F, 0.5F));
        tempMatrix.scale(new Vector3f(0.5F, 0.5F, 0.5F));
        Matrix4f.mul(tempMatrix, lightProjectionTemp, textureMatrix);
        Matrix4f.mul(textureMatrix, lightModelViewTemp, tempMatrix);
        Matrix4f.transpose(tempMatrix, textureMatrix);
    }

    /** Sets up the OpenGL states. */
	public void setUpOpenGL() {
	    int maxRenderbufferSize = glGetInteger(GL_MAX_RENDERBUFFER_SIZE_EXT);
	
	    if (!GLContext.getCapabilities().OpenGL14 && GLContext.getCapabilities().GL_ARB_shadow) {
	        System.out.println("Can't create shadows at all. Requires OpenGL 1.4 or the GL_ARB_shadow extension");
	        System.exit(0);
	    }
	
	    if (GLContext.getCapabilities().GL_ARB_shadow_ambient) {
	        ambientShadowsAvailable = true;
	    } else {
	        System.out.println("GL_ARB_shadow_ambient extension not availible.\n An extra rendering pass will be " +
	                "required.");
	    }
	
	    if (GLContext.getCapabilities().OpenGL20 || GLContext.getCapabilities().GL_EXT_framebuffer_object) {
	        System.out.println("Higher quality shadows are availible.");
	    }
	
	    maxTextureSize = glGetInteger(GL_MAX_TEXTURE_SIZE);
	
	    System.out.println("Maximum texture size: " + maxTextureSize);
	    System.out.println("Maximum renderbuffer size: " + maxRenderbufferSize);
	
	    /*
	       * Check to see if the maximum texture size is bigger than 2048.
	       * Performance drops too much if it much bigger than that.
	       */
	    if (maxTextureSize > 2048) {
	        maxTextureSize = 2048;
	        if (maxRenderbufferSize < maxTextureSize) {
	            maxTextureSize = maxRenderbufferSize;
	        }
	    }
	
	    if (useFBO) {
	        shadowWidth = maxTextureSize;
	        shadowHeight = maxTextureSize;
	    }
	
	    glClearColor(0.0F, 0.4F, 0.9F, 1.0F);
	
	    glEnable(GL_DEPTH_TEST);
	    glDepthFunc(GL_LEQUAL);
	    glPolygonOffset(factor, 0.0F);
	
	    glShadeModel(GL_SMOOTH);
	    glEnable(GL_LIGHTING);
	    glEnable(GL_COLOR_MATERIAL);
	    glEnable(GL_NORMALIZE);
	    glEnable(GL_LIGHT0);
	
	    // Setup some texture states
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	    glTexParameteri(GL_TEXTURE_2D, GL_DEPTH_TEXTURE_MODE, GL_INTENSITY);
	
	    // If ambient shadows are availible then we can skip a rendering pass.
	    if (ambientShadowsAvailable) {
	        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FAIL_VALUE_ARB, 0.5F);
	    }
	
	    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
	    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
	    glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
	    glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
	
	    // If we are using a FBO, we need to setup the framebuffer.
	    if (useFBO) {
	        frameBuffer = glGenFramebuffersEXT();
	        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frameBuffer);
	
	        renderBuffer = glGenRenderbuffersEXT();
	        glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, renderBuffer);
	
	        glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT32, maxTextureSize, maxTextureSize);
	
	        glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT,
	                renderBuffer);
	
	        glDrawBuffer(GL_NONE);
	        glReadBuffer(GL_NONE);
	
	        int FBOStatus = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
	        if (FBOStatus != GL_FRAMEBUFFER_COMPLETE_EXT) {
	            System.out.println("Framebuffer error!");
	        }
	
	        glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
	    }
	    generateShadowMap();
	}
	/** Render the scene, and then update. */
   public void render() {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(40, (float) Display.getWidth() / (float) Display.getHeight(), 1.0F, 1000.0F);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        GL11.glEnable(GL11.GL_CULL_FACE);
        GL11.glCullFace(GL11.GL_BACK);
        if(cameraDirY > 180)
        {
        	cameraDirY = -180 + (cameraDirY - 180);
        }
        if(cameraDirY < -180)
        {
        	cameraDirY = 180 + (cameraDirY + 180);
        }
        //gluLookAt(cameraPosition.get(0), cameraPosition.get(1), cameraPosition.get(2), 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
               // 0.0F);
        // TODO Add cam.applyModelViewMatrix(true) here and remove the 2
        // lines above
        GL11.glTranslated(0, 0, -10);
        GL11.glRotated(cameraDirY,0 ,-1, 0);
        GL11.glRotated(cameraDirX,(float)(-Math.cos(Math.toRadians(cameraDirY))),0, (float)(Math.sin(Math.toRadians(cameraDirY))));
        GL11.glTranslated(-cameraX, -cameraY, -cameraZ);
        glViewport(0, 0, Display.getWidth(), Display.getHeight());

        glLight(GL_LIGHT0, GL_POSITION, lightPosition);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (showShadowMap) {
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glMatrixMode(GL_TEXTURE);
            glPushMatrix();
            glLoadIdentity();
            glEnable(GL_TEXTURE_2D);
            glDisable(GL_LIGHTING);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);

            glBegin(GL_QUADS);
            glTexCoord2f(0.0F, 0.0F);
            glVertex2f(-1.0F, -1.0F);
            glTexCoord2f(1.0F, 0.0F);
            glVertex2f(1.0F, -1.0F);
            glTexCoord2f(1.0F, 1.0F);
            glVertex2f(1.0F, 1.0F);
            glTexCoord2f(0.0F, 1.0F);
            glVertex2f(-1.0F, 1.0F);
            glEnd();

            glDisable(GL_TEXTURE_2D);
            glEnable(GL_LIGHTING);
            glPopMatrix();
            glMatrixMode(GL_PROJECTION);
            gluPerspective(45.0F, 1.0F, 1.0F, 1000.0F);
            glMatrixMode(GL_MODELVIEW);
        } else {
            /*
                    * If we dont have the ambient shadow extention, we will need to
                    * add an extra rendering pass.
                    */
            if (!ambientShadowsAvailable) {
                FloatBuffer lowAmbient = BufferUtils.createFloatBuffer(4);
                lowAmbient.put(new float[]{0.1F, 0.1F, 0.1F, 1.0F});
                lowAmbient.flip();

                FloatBuffer lowDiffuse = BufferUtils.createFloatBuffer(4);
                lowDiffuse.put(new float[]{0.35F, 0.35F, 0.35F, 1.0F});
                lowDiffuse.flip();

                glLight(GL_LIGHT0, GL_AMBIENT, lowAmbient);
                glLight(GL_LIGHT0, GL_DIFFUSE, lowDiffuse);

                renderObjects(true);

                glAlphaFunc(GL_GREATER, 0.9F);
                glEnable(GL_ALPHA_TEST);
            }

            glLight(GL_LIGHT0, GL_AMBIENT, ambientLight);
            glLight(GL_LIGHT0, GL_DIFFUSE, diffuseLight);

            glEnable(GL_TEXTURE_2D);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

            glEnable(GL_TEXTURE_GEN_S);
            glEnable(GL_TEXTURE_GEN_T);
            glEnable(GL_TEXTURE_GEN_R);
            glEnable(GL_TEXTURE_GEN_Q);

            tempBuffer.put(0, textureMatrix.m00);
            tempBuffer.put(1, textureMatrix.m01);
            tempBuffer.put(2, textureMatrix.m02);
            tempBuffer.put(3, textureMatrix.m03);

            glTexGen(GL_S, GL_EYE_PLANE, tempBuffer);

            tempBuffer.put(0, textureMatrix.m10);
            tempBuffer.put(1, textureMatrix.m11);
            tempBuffer.put(2, textureMatrix.m12);
            tempBuffer.put(3, textureMatrix.m13);

            glTexGen(GL_T, GL_EYE_PLANE, tempBuffer);

            tempBuffer.put(0, textureMatrix.m20);
            tempBuffer.put(1, textureMatrix.m21);
            tempBuffer.put(2, textureMatrix.m22);
            tempBuffer.put(3, textureMatrix.m23);

            glTexGen(GL_R, GL_EYE_PLANE, tempBuffer);

            tempBuffer.put(0, textureMatrix.m30);
            tempBuffer.put(1, textureMatrix.m31);
            tempBuffer.put(2, textureMatrix.m32);
            tempBuffer.put(3, textureMatrix.m33);

            glTexGen(GL_Q, GL_EYE_PLANE, tempBuffer);
            

            
            renderObjects(true);

            glDisable(GL_ALPHA_TEST);
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_TEXTURE_GEN_S);
            glDisable(GL_TEXTURE_GEN_T);
            glDisable(GL_TEXTURE_GEN_R);
            glDisable(GL_TEXTURE_GEN_Q);
        }

        if (glGetError() != GL_NO_ERROR) {
           // System.out.println("An OpenGL error occurred");
        }
    }

    /** Handles the keyboard and mouse input. */
   public void input() {
       
        if(Keyboard.isKeyDown(Keyboard.KEY_T))
        {
        	showShadowMap = true;
        }
        else
        {
        	showShadowMap = false;
        }

        // TODO Add cam.handleMouse()
        // TODO Add cam.handleKeyboard()
    }

    /** Cleanup after the program. */
    private static void cleanUp() {
        glDeleteFramebuffersEXT(frameBuffer);
        glDeleteRenderbuffersEXT(renderBuffer);
        Display.destroy();
    }

    /** Sets up the FloatBuffers to be used later on. */
    private static void setUpBufferValues() {
        ambientLight.put(new float[]{0.2F, 0.2F, 0.2F, 1.0F});
        ambientLight.flip();

        diffuseLight.put(new float[]{0.7F, 0.7F, 0.7F, 1.0F});
        diffuseLight.flip();

        cameraPosition.put(new float[]{100.0F, 50.0F, 200.0F, 1.0F});
        cameraPosition.flip();

        lightPosition.put(new float[]{20.0F, 100.0F, 20.0F, 1.0F});
        lightPosition.flip();
    }
}