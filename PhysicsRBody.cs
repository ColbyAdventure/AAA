using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsRBody : MonoBehaviour
{
    public float mass = 1f;                             // Mass of the RigidBody
    public float bounciness = 1;                        // The bounciness factor (value between 0 and 1, 0 being no bounce, and 1 being super bouncy!) the reflection Co-efficient
    public bool obeysGravity = true;                    // Whether or not this body obeys gravity
    public Vector2 gravity = new Vector2(0, -9.8f);     // The gravity vector applied to this body

    public Vector2 currentVelocity;                     // The current velocity the body is moving at
    public Vector2 maxVelocity = new Vector2(10f, 10f); // The maximum allowed velocity for this object

    public bool grounded;                               // Toggle is the object is Grounded (With only the gravity and not velocity effecting it) 

    private Vector2 totalForces;                        // The end result of all the forces affecting the object, expressed as a [x,y]
    private PhysicsEngine engine;                       // An instance of the PhysicsEngine called engine

    public struct AABB
    {
        public Vector2 bLeft;                           // a [x,y] representing the bottom left of the object
        public Vector2 tRight;                          // a [x,y] representing the Top Right of the object
    }

    public AABB aabb;                                   // A publicly availible copy of a AABB called aabb;


    public void AddForce(Vector2 force)
    {
        //You gotta running total of forces so far... Add this new force into the mix
        //Stored as [x,y]
        totalForces += force;
    }

    public void Stop()
    {
        //Reset the variables used to keep track of Velocity and the Total Forces 
        //"Zeroing Everthing Out"
        currentVelocity = Vector2.zero;
        totalForces = Vector2.zero;
    }

    public bool IsGrounded()
    {
        //********************
        //the local variable is set to the same as the bool in "engine" 
        //called IsGrounded about the object this script is attached to
        //Asks "Is this grounded? True or False?"  
        grounded = engine.IsGrounded(this);
        return grounded;
    }

    /*
     * ______________ <-What does this function do?
     */
    void SetAABB()
    {
        //creates an Instance of a Bounds called 'bound' with the first [x,y] as the Bottom Left
        //and the second [x,y] as the top right

        Bounds bound = new Bounds(new Vector2(0, 0), new Vector2(1, 1));
        Renderer renderer = GetComponent<Renderer>();

        if (renderer)
        {
            bound = renderer.bounds;
        }

        //*********
        //Assigning the x and y to the [x,y] of bLeft and tRight
        //The members of the struct aabb we invented on line 20

        aabb.bLeft = new Vector2(bound.center.x - bound.extents.x, bound.center.y - bound.extents.y);
        aabb.tRight = new Vector2(bound.center.x + bound.extents.x, bound.center.y + bound.extents.y);
    }

    void Start(){
        //When this script first runs this function is ran (Like the Main of a normal program in c++ or Whatever)
        //Runs the SetAABB Function Once
        //Finds one Object in Game Land with the Hashtag "PhysicsEngine"
        //Finds the Component (Like a member) called <PhysicsEngine>
        //Uses the PhysicsEngine attached to the Object to add a Rigidbody to the Attached (To the Script) object

        SetAABB();
        engine = GameObject.FindWithTag("PhysicsEngine").GetComponent<PhysicsEngine>();

        engine.AddRigidBody(this);
    }

    /*
     * ______________ Describe how this function works
     */
    public void Integrate(float dT){
        //Make a new [x,y] called acceleration

        Vector2 acceleration = new Vector2();

        /// 
        /// ______________ What is the purpose of this part of code?
        /// 
        //First, check if "obeysGravity" is true and "IsGrounded" is false
        //If both conditions are met, make acceleration [x,y] to gravity [0,-9.8]
        //Otherwise 
        //if currentVelocity on the y is less than 0.05f, set the currentVelocity.y = 0
        if (obeysGravity && !IsGrounded()){ 
            acceleration = gravity;
        }
        else{
            if (Mathf.Abs(currentVelocity.y) < 0.05f) currentVelocity.y = 0;
        }
        ///
        ///
        ///

        //acceleration [x,y] = [totalForces.x/mass, totalForces.y/mass]
        //if mass = 0, zero out the acceleration vector [0,0]
        acceleration += totalForces / mass;
        if (mass == 0)
            acceleration = Vector2.zero;
        
        //**************
        //Current velocity + acceleration * Delta T
        currentVelocity += acceleration * dT;


        //A Temporary Vector2 [x,y] to hold the object's current position
        //temp.x += (currentVelocity.x * Delta T)
        //temp.y += (currentVelocity.y * Delta T)
        //So... 
        //the new position for the transform is copied from the temp [x,y]
        //Call The SetAABB function,
        //Then Zero Out the totalForces [x,y]
        Vector2 temp = transform.position;
        temp += currentVelocity * dT;
        transform.position = temp;
        SetAABB();

        totalForces = Vector2.zero;
    }
}
