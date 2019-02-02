using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//Galal's Original Comments Are ////
//Check the ****
//Fill In the ...
//Attached As a Component on page 31 -36 of week 3 slides
//https://www.youtube.com/watch?v=ttHzAOn2gEo
//https://www.youtube.com/channel/UCzQvgRgjjxhzEORvefubDPw
//https://www.youtube.com/watch?v=l0KpQY9-YdM


public class PhysicsEngine : MonoBehaviour
{
    //The amount of "Close enough" that is Close enough
    public float groundedTol = 0.1f;

    public struct CollisionPair{
        //Creates an object called CollisionPair, Containing the two homemade Rigidbodies we bangin together 
        public PhysicsRBody rigidBodyA;
        public PhysicsRBody rigidBodyB;
    }

    public struct CollisionInfo{
        //Creates an object called CollisionInfo, Containing  a [x,y] for the collisionNormal
        //And a float (Number With Decimals) expressing a penetration
        public Vector2 collisionNormal;
        public float penetration;
    }
    //Creates a Dictionary (Thing means Thing) of collisions
    //First Spot's CollisionPair, Second Spot's CollisionInfo
    private Dictionary<CollisionPair, CollisionInfo> collisions = new Dictionary<CollisionPair, CollisionInfo>();

    //Creates a List (like... a list for listing?)
    private List<PhysicsRBody> rigidBodies = new List<PhysicsRBody>();

    public void AddRigidBody(PhysicsRBody rigidBody){
        //Add a PhysicsRBody called rigidBody to the List of "PhysicsRBody"s called rigidBodies
        //WHY DO YOU WANT TO MAKE ME CRY??? Wtf?!? :)
        rigidBodies.Add(rigidBody);
    }

    void IntegrateBodies(float dT){
        //For Each of the rigidbBodies in that List From ______ 
        //************

        foreach (PhysicsRBody rb in rigidBodies){
            rb.Integrate(dT);
        }
    }
    //Testing for "isGrounded"
    public bool IsGrounded(PhysicsRBody rigidBody){
        //Out of that list we made,
        //Compare rigidBody and rb (Call it "The other One" for convience)
        //Line ___ are they the same? -> return False
        //Is the X of rigidBody.aabb.bLeft more left than the X of rb.aabb.tRight &&AND&&
        //Is the X of rigidBody.aabb.tRight.x more Right than the X of rb.aabb.bLeft.x &&AND&&
        //Is the movement on the y axis (rigidBody.currentVelocity.y) smaller than the tolerance
        //If it (*What?**) passes all that, the mighty IsGrounded Bool Returns In triumph as true!
        //Fail any of the trials, and return false, like a birch
        foreach (PhysicsRBody rb in rigidBodies){
            if (rb != rigidBody){
                if (rigidBody.aabb.bLeft.x < rb.aabb.tRight.x 
                    && rigidBody.aabb.tRight.x > rb.aabb.bLeft.x
                    && Mathf.Abs(rigidBody.aabb.bLeft.y - rb.aabb.tRight.y) <= groundedTol){
                    if (Mathf.Abs(rigidBody.currentVelocity.y) < groundedTol)
                        return true;
                }
            }
        }
        return false;
    }

    void CheckCollisions(){
        //Check each PhysicsRBody against Every other PhysicsRBody
        //check to make sure they aren't the Same PhysicsRBody
        //Build a CollisionPair called pair
        //Build a CollisionInfo called colInfo
        //Distance is a Vector2 [(bodyB.x -bodyA.x), (bodyB.y -bodyA.y)]
        //halfSizeA is a Vector2 [Half the length of the top/bottom, Half the length of the sides]
        //halfSizeB is a Vector2 [Half the length of the top/bottom, Half the length of the sides]
        //Kinda Like a Radius, but fer a square
        //then line ____ happens.. Vector2 gap =
        //
        foreach (PhysicsRBody bodyA in rigidBodies.GetRange(0, rigidBodies.Count - 1)){
            foreach (PhysicsRBody bodyB in rigidBodies.GetRange(rigidBodies.IndexOf(bodyA), rigidBodies.Count - rigidBodies.IndexOf(bodyA))){
                if (bodyA != bodyB){
                    CollisionPair pair = new CollisionPair();
                    CollisionInfo colInfo = new CollisionInfo();
                    pair.rigidBodyA = bodyA; pair.rigidBodyB = bodyB;

                    Vector2 distance = bodyB.transform.position - bodyA.transform.position;

                    Vector2 halfSizeA = (bodyA.aabb.tRight - bodyA.aabb.bLeft) / 2;
                    Vector2 halfSizeB = (bodyB.aabb.tRight - bodyB.aabb.bLeft) / 2;

                    Vector2 gap = new Vector2(Mathf.Abs(distance.x), Mathf.Abs(distance.y)) - (halfSizeA + halfSizeB);

                    //// Seperating Axis Theorem test
                    if (gap.x < 0 && gap.y < 0){
                        //Check if the the two objects overlay on both the x AND y axis
                        Debug.Log("Collided!!!");

                        if (collisions.ContainsKey(pair)){
                        //If the collisions list contains this "pair", remove it from the list
                        //Removed cuz They Collided, so we don't need to check em any more
                            collisions.Remove(pair);
                        }

                        if (gap.x > gap.y){
                            if (distance.x > 0){
                            //So There is space between the x positions, hmm?
                                //// ... Update collision normal
                            }
                            else{
                            //So There is ain't a space between the x positions
                                //// ... Update collision normal
                            }
                            //The Magic gap.x Calcuation is added to the penetration member of colInfo
                            colInfo.penetration = gap.x;    
                        }
                        else{
                            if (distance.y > 0){
                            //So There is space between the y positions, hmm?
                                //// ... Update collision normal
                            }
                            else
                            {
                            //So There is ain't a space between the y positions
                                //// ... Update collision normal
                            }
                            //The Magic gap.y Calcuation is added to the penetration member of colInfo
                            //Completing it's Might!
                            colInfo.penetration = gap.y; 
                        }
                        //Adding this pair, alomg with the colInfo to the Collisions list
                        collisions.Add(pair, colInfo);
                    }
                    else if (collisions.ContainsKey(pair)){
                        //If the collisions list contains this "pair", remove it from the list
                        //Removed cuz They Collided?***********, so we don't need to check em any more
                        Debug.Log("removed");
                        collisions.Remove(pair);
                    }

                }
            }
        }
    }

    void ResolveCollisions(){
        foreach (CollisionPair pair in collisions.Keys){
            //For each pair in the collisions list
            //Check both the rigidBodys, and uses the lower of the two as minBounce
            //Calculate the Velocity Along the Normal*******
            //If velAlongNormal is bigger than 0, continue the function
            float minBounce = Mathf.Min(pair.rigidBodyA.bounciness, pair.rigidBodyB.bounciness);
            float velAlongNormal = Vector2.Dot(pair.rigidBodyB.currentVelocity - pair.rigidBodyA.currentVelocity, collisions[pair].collisionNormal);
            if (velAlongNormal > 0) continue;

            //Make j = (1.8) * velAlongNormal
            //If rigidBody A or B has zero Mass, it's Inverse is also 0
            //Otherwise invMassA/B is 1/mass
            //J = the Sum of invMassA + invMassB
            //Impulse = J times this "pair"s collisionNormal
            float j = -(1 + minBounce) * velAlongNormal;
            float invMassA, invMassB;
            if (pair.rigidBodyA.mass == 0)
                invMassA = 0;
            else
                invMassA = 1 / pair.rigidBodyA.mass;

            if (pair.rigidBodyB.mass == 0)
                invMassB = 0;
            else
                invMassB = 1 / pair.rigidBodyB.mass;

            j /= invMassA + invMassB;

            Vector2 impulse = j * collisions[pair].collisionNormal;

            //// ... update velocities
            ////*************************
            //Basically Original Postion + Impulse

            if (Mathf.Abs(collisions[pair].penetration) > 0.01f){
            //Check if the pair is overlapping More than tollerance
            //run The PositionalCorrection Function
                PositionalCorrection(pair);
            }
        }
    }

    /*
    * ______________ Why do we need this function? 
    * ______________ Try taking it out and see what happens
    */
    void PositionalCorrection(CollisionPair c){
        //If the shapes are in fact Colliding, push them apart the shortest distance direction (Away from the collision)
        //Check if The Mass of either Body is 0, and set their invMass to 0 if true
        //Otherwise it's 1/their respective Masses (or the Inverse, see where it went there?)
        const float percent = 0.2f;
                            
        float invMassA, invMassB;
        if (c.rigidBodyA.mass == 0)
            invMassA = 0;
        else
            invMassA = 1 / c.rigidBodyA.mass;

        if (c.rigidBodyB.mass == 0)
            invMassB = 0;
        else
            invMassB = 1 / c.rigidBodyB.mass;
        //The position correction vector2 [x,y]
        //Basically, the (penetration of the collision/The combined Inverse Masses * percent) in the direction opposite of the collision
        Vector2 correction = ((collisions[c].penetration / (invMassA + invMassB)) * percent) * -collisions[c].collisionNormal;

        //Take the original position of the Rigidbody and add the correction to it
        Vector2 temp = c.rigidBodyA.transform.position;
        temp -= invMassA * correction;
        c.rigidBodyA.transform.position = temp;

        //Take the original position of the Rigidbody and add the correction to it
        temp = c.rigidBodyB.transform.position;
        temp += invMassB * correction;
        c.rigidBodyB.transform.position = temp;
    }

    void UpdatePhysics(){
        //// .... 
        //CheckCollisions();
        //ResolveCollisions();
        //PositionalCorrection();
    }

    // Update is called once per frame
    void FixedUpdate(){
        UpdatePhysics();
    }
}