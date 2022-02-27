

public class ConstraintVec {
	javax.vecmath.Vector3f originA = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f originB = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f linearLow = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f linearUp = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f angleLow = new javax.vecmath.Vector3f();
	javax.vecmath.Vector3f angleUp = new javax.vecmath.Vector3f();
	public void setOrigins(float aX, float aY, float aZ, float bX, float bY, float bZ)
	{
		this.originA.x = aX;
		this.originA.y = aY;
		this.originA.z = aZ;
		
		this.originB.x = bX;
		this.originB.y = bY;
		this.originB.z = bZ;
	}
	public void setLinears(float aX, float aY, float aZ, float bX, float bY, float bZ)
	{
		this.linearLow.x = aX;
		this.linearLow.y = aY;
		this.linearLow.z = aZ;
		
		this.linearUp.x = bX;
		this.linearUp.y = bY;
		this.linearUp.z = bZ;
	}
	public void setAngles(float aX, float aY, float aZ, float bX, float bY, float bZ)
	{
		if(aX != 1 && aY != 1 && aZ != 1 && bX != 1 && bY != 1 && bZ != 1)
		{
			this.angleLow.x = (float)Math.toRadians(aX);
			this.angleLow.y = (float)Math.toRadians(aY);
			this.angleLow.z = (float)Math.toRadians(aZ);
			
			this.angleUp.x = (float)Math.toRadians(bX);
			this.angleUp.y = (float)Math.toRadians(bY);
			this.angleUp.z = (float)Math.toRadians(bZ);
		}
		else
		{
			this.angleLow.x = aX;
			this.angleLow.y = aY;
			this.angleLow.z = aZ;
			
			this.angleUp.x = bX;
			this.angleUp.y = bY;
			this.angleUp.z = bZ;
		}
	}
}
