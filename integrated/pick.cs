//using System.Collections.Generic;
//using UnityEngine;
//using Unity.MLAgents;
//using Unity.MLAgents.Sensors;
//using Unity.MLAgents.Actuators;
//using static UnityEngine.GraphicsBuffer;
//using System.Collections;

//public class PicknPlace : Agent
//{
//    public HingeJoint arm1Joint; // Hinge joint for the first arm
//    public HingeJoint arm2Joint; // Hinge joint for the second arm
//    public Transform targetPosition;
//    public Rigidbody endEffector;
//    public Rigidbody Target;
//    public Rigidbody dummy;

//    private Vector3 initialArm1Position;
//    private Quaternion initialArm1Rotation;

//    private Vector3 initialArm2Position;
//    private Quaternion initialArm2Rotation;

//    private Vector3 initialEndEffectorPosition;
//    private Quaternion initialEndEffectorRotation;

//    private Vector3 initialTargetPosition;
//    private Quaternion initialTargetRotation;

//    public Rigidbody arm1;
//    public Rigidbody arm2;

//    public Rigidbody table;

//    public Rigidbody mobileBase;


//    public float motorForce = 100f; // Force applied by the motors
//    public float maxMotorVelocity = 100f; // Maximum velocity for the motors

//    public float arm1MinLimit = 0f; // Minimum angle for arm1
//    public float arm1MaxLimit = 90f; // Maximum angle for arm1
//    public float arm2MinLimit = -45f; // Minimum angle for arm2
//    public float arm2MaxLimit = 60f; // Maximum angle for arm2

//    public float targetArm1Angle = 0f;
//    public float targetArm2Angle = 0f;

//    //private bool targetReached = false; // Flag to track if the target was reached
//    public float toppleAngleThreshold = 45f; // Maximum allowed tilt angle in degrees

//    public bool targetReached = false;
//    public bool tablehit = false;
//    public bool ArmxTarget = false;
//    public bool ArmxTable = false;

//    //private float episodeStartTime; // Stores the start time of the episode
//    //public float maxEpisodeDuration = 100000f;
//    private int currentStepCount = 0; // Tracks the number of steps in the episode
//    public int maxStepsPerEpisode = 20000; // Set the maximum steps per episode

//    //public int eps = 0; //No of completed eps

//    void Start()
//    {
//        // Set limits for arm1
//        JointLimits arm1Limits = arm1Joint.limits;
//        arm1Limits.min = arm1MinLimit;
//        arm1Limits.max = arm1MaxLimit;
//        arm1Joint.limits = arm1Limits;
//        arm1Joint.useLimits = true;

//        // Set limits for arm2
//        JointLimits arm2Limits = arm2Joint.limits;
//        arm2Limits.min = arm2MinLimit;
//        arm2Limits.max = arm2MaxLimit;
//        arm2Joint.limits = arm2Limits;
//        arm2Joint.useLimits = true;


//        initialArm1Position = arm1.transform.position;
//        initialArm1Rotation = arm1.transform.rotation;

//        // Store initial states of arm2
//        initialArm2Position = arm2.transform.position;
//        initialArm2Rotation = arm2.transform.rotation;

//        // Store initial states of end effector
//        initialEndEffectorPosition = endEffector.transform.position;
//        initialEndEffectorRotation = endEffector.transform.rotation;

//        initialTargetPosition = Target.transform.position;
//        initialTargetRotation = Target.transform.rotation;

//    }

//    //void Start()
//    //{

//    //    Rigidbody endEffectorRb = endEffector.GetComponent<Rigidbody>();
//    //    endEffectorRb.constraints = RigidbodyConstraints.FreezeRotation;  // Prevent rotation of end effector
//    //}



//    public override void OnActionReceived(ActionBuffers actions)
//    {
//        float arm1Action = actions.ContinuousActions[0];
//        float arm2Action = actions.ContinuousActions[1];

//        JointMotor arm1Motor = arm1Joint.motor;
//        arm1Motor.targetVelocity = arm1Action * maxMotorVelocity;
//        //arm1Motor.targetVelocity = arm1Action;
//        arm1Motor.force = motorForce;
//        arm1Joint.motor = arm1Motor;
//        arm1Joint.useMotor = true;

//        JointMotor arm2Motor = arm2Joint.motor;
//        arm2Motor.targetVelocity = arm2Action * maxMotorVelocity;
//        //arm2Motor.targetVelocity = arm2Action;
//        arm2Motor.force = motorForce;
//        arm2Joint.motor = arm2Motor;
//        arm2Joint.useMotor = true;

//        currentStepCount++; // Increment step counter

//        if (currentStepCount >= maxStepsPerEpisode)
//        {
//            //Debug.Log("Reached max steps for this episode. Ending episode.");

//            EndEpisode();

//        }




//        if (targetReached == true)
//        {
//            AddReward(1.0f);
//            EndEpisode(); // End the episode
//        }



//        // Coroutine to smoothly reset the arm and then end the episod


//        if (tablehit == true)
//        {
//            AddReward(-0.01f);

//            EndEpisode();

//            //Debug.Log("Episode has been ended.");
//        }

//        //if (IsTargetToppled())
//        //{
//        //AddReward(-1.0f);

//        //EndEpisode();

//        //}

//        //if (Time.time - episodeStartTime > maxEpisodeDuration)
//        //{
//        //Debug.Log("Episode timed out. Ending episode.");
//        //EndEpisode();
//        //}

//        if (ArmxTarget == true)
//        {
//            AddReward(-0.0001f);
//            EndEpisode();

//            //Debug.Log("Episode has been ended.");
//        }

//        if (ArmxTable == true)
//        {
//            EndEpisode();
//            //Debug.Log("Episode has been ended.");
//        }

//    }

//    public void EndEpisodeOnTargetReached()
//    {
//        //Debug.Log("End effector reached the target!");
//        //AddReward(1.0f);  // Reward for reaching the target
//        //EndEpisode();     // End the episode
//        //Debug.Log("Episode has been ended.");
//        targetReached = true;

//    }

//    public void EndEpisodeOnHittingTable()
//    {
//        //Debug.Log("End effector reached the target!");
//        //AddReward(1.0f);  // Reward for reaching the target
//        //EndEpisode();     // End the episode
//        //Debug.Log("Episode has been ended.");
//        tablehit = true;

//    }

//    public void ArmHitTarget()
//    {
//        ArmxTarget = true;
//    }

//    public void ArmHitTable()
//    {
//        ArmxTable = true;
//    }



//    public override void Heuristic(in ActionBuffers actionsOut)
//    {
//        var continuousActionsOut = actionsOut.ContinuousActions;
//        continuousActionsOut[0] = Input.GetAxis("Horizontal"); // Horizontal axis
//        continuousActionsOut[1] = Input.GetAxis("Vertical");   // Vertical axis

//    }

//    public override void CollectObservations(VectorSensor sensor)
//    {
//        // Add joint positions and target position as observations
//        sensor.AddObservation(arm1Joint.angle); // Current angle of arm1Joint
//        sensor.AddObservation(arm2Joint.angle); // Current angle of arm2Joint
//        sensor.AddObservation(targetPosition.position); // Target position

//    }

//    public override void OnEpisodeBegin()
//    {
//        //targetReached = false; // Reset the target reached flag
//        //Debug.Log("New Episode has started.");
//        //ResetArmJoints();

//        //if (dummy != null)
//        //{
//        //    dummy.gameObject.SetActive(false); // Deactivate the dummy's GameObject
//        //}

//        //if (Target != null)
//        //{
//        //    Target.gameObject.SetActive(true); // Reactivate the target's GameObject
//        //}




//        Debug.Log($"Steps in the previous episode: {currentStepCount}");
//        currentStepCount = 0; // Reset step counter at the start of each episode

//        if (targetReached) // A flag you set when the goal is reached
//        {
//            // Reset arm1
//            arm1.transform.position = initialArm1Position;
//            arm1.transform.rotation = initialArm1Rotation;

//            //Rigidbody arm1Rb = arm1.GetComponent<Rigidbody>();
//            if (arm1 != null)
//            {
//                arm1.linearVelocity = Vector3.zero;
//                arm1.angularVelocity = Vector3.zero;
//                arm1.useGravity = false; // Disable gravity
//                arm1.constraints = RigidbodyConstraints.FreezePositionY; // Freeze necessary axes
//            }

//            // Reset arm2
//            arm2.transform.position = initialArm2Position;
//            arm2.transform.rotation = initialArm2Rotation;

//            //Rigidbody arm2Rb = arm2.GetComponent<Rigidbody>();
//            if (arm2 != null)
//            {
//                arm2.linearVelocity = Vector3.zero;
//                arm2.angularVelocity = Vector3.zero;
//                arm2.useGravity = false; // Disable gravity
//                arm2.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation; // Freeze necessary axes
//            }

//            // Reset the target
//            Target.transform.position = initialTargetPosition;
//            Target.transform.rotation = initialTargetRotation;

//            //Rigidbody targetRb = Target.GetComponent<Rigidbody>();
//            if (Target != null)
//            {
//                Target.linearVelocity = Vector3.zero;
//                Target.angularVelocity = Vector3.zero;
//                Target.useGravity = false; // Disable gravity
//            }

//            // Reset end-effector
//            endEffector.transform.position = initialEndEffectorPosition;
//            endEffector.transform.rotation = initialEndEffectorRotation;

//            //Rigidbody eeRb = endEffector.GetComponent<Rigidbody>();
//            if (endEffector != null)
//            {
//                endEffector.linearVelocity = Vector3.zero;
//                endEffector.angularVelocity = Vector3.zero;
//                endEffector.useGravity = false; // Disable gravity
//                endEffector.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation; // Freeze necessary axes
//            }

//            if (dummy != null)
//            {
//                dummy.linearVelocity = Vector3.zero;
//                dummy.angularVelocity = Vector3.zero;
//                dummy.useGravity = false; // Disable gravity
//                dummy.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation;
//            }


//            // Optionally stop the agent from further actions
//            this.enabled = false; // Prevents further execution
//            Debug.Log("Agent and its components are reset and gravity disabled.");
//        }

//        arm1.transform.position = initialArm1Position;
//        arm1.transform.rotation = initialArm1Rotation;
//        arm1.linearVelocity = Vector3.zero;
//        arm1.angularVelocity = Vector3.zero;

//        // Reset arm2
//        arm2.transform.position = initialArm2Position;
//        arm2.transform.rotation = initialArm2Rotation;
//        arm2.linearVelocity = Vector3.zero;
//        arm2.angularVelocity = Vector3.zero;
//        //ResetTargetPosition();

//        Target.transform.position = initialTargetPosition;
//        Target.transform.rotation = initialTargetRotation;
//        Target.linearVelocity = Vector3.zero;
//        Target.angularVelocity = Vector3.zero;

//        endEffector.transform.position = initialEndEffectorPosition;
//        endEffector.transform.rotation = initialEndEffectorRotation;
//        endEffector.linearVelocity = Vector3.zero;
//        endEffector.angularVelocity = Vector3.zero;

//        targetReached = false;
//        tablehit = false;
//        ArmxTarget = false;
//        ArmxTable = false;

//    }

//    //private void ResetArmJoints()
//    //{
//    //    JointMotor motor = arm1Joint.motor;
//    //    motor.targetVelocity = 0.001f;  // Ensure the joint stays still
//    //    motor.force = 50f;
//    //    arm1Joint.motor = motor;
//    //    arm1Joint.useMotor = true;

//    //    JointMotor motor2 = arm2Joint.motor;
//    //    motor2.targetVelocity = 0.001f;  // Ensure the joint stays still
//    //    motor2.force = 50f;
//    //    arm2Joint.motor = motor2;
//    //    arm2Joint.useMotor = true;
//    //}

//    private bool IsTargetToppled()
//    {
//        // Calculate the angle between the target's "up" direction and the world "up" direction
//        float angle = Vector3.Angle(Target.transform.up, Vector3.up);

//        // Check if the angle exceeds the threshold
//        return angle > toppleAngleThreshold;
//    }

//    //private void SetJointMotor(HingeJoint joint, float targetAngle)
//    //{
//    //    var motor = joint.motor;
//    //    motor.targetVelocity = (targetAngle - joint.angle) * 10f;
//    //    motor.force = 10f;
//    //    motor.freeSpin = false;
//    //    joint.motor = motor;
//    //    joint.useMotor = true;
//    //}

//    //private void ResetTargetPosition()
//    //{
//    //    // Reset the target position (you can randomize this if desired)
//    //    targetPosition.position = new Vector3(
//    //        2f,// Randomize x
//    //        0.4f,//andomize y
//    //        0.7f//andomize z
//    //    );

//    //    //Debug.Log($"Target position reset to: {targetPosition.position}");
//    //}


//    //private bool TouchesTarget()
//    //{
//    //    // Get the bounds of the base's collider
//    //    Collider endEffectorCollider = endEffector.GetComponent<Collider>();
//    //    //if (baseCollider == null)
//    //    //{
//    //    //    Debug.LogError("Base does not have a Collider attached!");
//    //    //    return false;
//    //    //}

//    //    // Array of all obstacles
//    //    Collider targetCollider = Target.GetComponent<Collider>();



//    //    // Check if the base's collider intersects with the obstacle's collider
//    //    if (endEffectorCollider.bounds.Intersects(targetCollider.bounds))
//    //    {
//    //        return true; // Collision detected
//    //    }


//    //    return false; // No collisions
//    //}


//}

using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using static UnityEngine.GraphicsBuffer;
using System.Collections;

public class PicknPlace : Agent
{
    public HingeJoint arm1Joint; // Hinge joint for the first arm
    public HingeJoint arm2Joint; // Hinge joint for the second arm
    public Transform targetPosition;
    public Rigidbody endEffector;
    public Rigidbody Target;
    public Rigidbody dummy;
    public task1 agent0;
    private bool ispickActive = false;

    private Vector3 initialArm1Position;
    private Quaternion initialArm1Rotation;

    private Vector3 initialArm2Position;
    private Quaternion initialArm2Rotation;

    private Vector3 initialEndEffectorPosition;
    private Quaternion initialEndEffectorRotation;

    private Vector3 initialTargetPosition;
    private Quaternion initialTargetRotation;

    private Vector3 finalArm2Position;

    public Rigidbody arm1;
    public Rigidbody arm2;

    public Rigidbody table;

    public Rigidbody mobileBase;


    public float motorForce = 100f; // Force applied by the motors
    public float maxMotorVelocity = 100f; // Maximum velocity for the motors

    public float arm1MinLimit = 0f; // Minimum angle for arm1
    public float arm1MaxLimit = 90f; // Maximum angle for arm1
    public float arm2MinLimit = -45f; // Minimum angle for arm2
    public float arm2MaxLimit = 60f; // Maximum angle for arm2

    public float targetArm1Angle = 0f;
    public float targetArm2Angle = 0f;

    //private bool targetReached = false; // Flag to track if the target was reached
    public float toppleAngleThreshold = 45f; // Maximum allowed tilt angle in degrees

    public bool targetReached = false;
    public bool tablehit = false;
    public bool ArmxTarget = false;
    public bool ArmxTable = false;

    //private float episodeStartTime; // Stores the start time of the episode
    //public float maxEpisodeDuration = 100000f;
    private int currentStepCount = 0; // Tracks the number of steps in the episode
    public int maxStepsPerEpisode = 20000; // Set the maximum steps per episode

    //public int eps = 0; //No of completed eps

    void Start()
    {
        // Set limits for arm1
        JointLimits arm1Limits = arm1Joint.limits;
        arm1Limits.min = arm1MinLimit;
        arm1Limits.max = arm1MaxLimit;
        arm1Joint.limits = arm1Limits;
        arm1Joint.useLimits = true;

        // Set limits for arm2
        JointLimits arm2Limits = arm2Joint.limits;
        arm2Limits.min = arm2MinLimit;
        arm2Limits.max = arm2MaxLimit;
        arm2Joint.limits = arm2Limits;
        arm2Joint.useLimits = true;


        initialArm1Position = arm1.transform.position;
        initialArm1Rotation = arm1.transform.rotation;

        // Store initial states of arm2
        initialArm2Position = new Vector3(2f, 0.7f, 0.2f);
        initialArm2Rotation = Quaternion.Euler(90f, 0f, 0f);
        finalArm2Position = new Vector3(2f, 3.5f, 0.2f);

        // Store initial states of end effector
        initialEndEffectorPosition = endEffector.transform.position;
        initialEndEffectorRotation = endEffector.transform.rotation;

        initialTargetPosition = Target.transform.position;
        initialTargetRotation = Target.transform.rotation;

        Debug.Log($"{initialArm2Position}");
        Debug.Log($"{initialArm2Rotation}");

        //if (!agent0.GetComponent<task1>().task1done)
        //{
        //    this.enabled = false;
        //    Debug.Log("Second Agent is not enabled");
        //}

        if (agent0.GetComponent<task1>().task1done)
        {
            this.enabled = true;
            Debug.Log("Second Agent Activated");
        }


    }

    //void Start()
    //{

    //    Rigidbody endEffectorRb = endEffector.GetComponent<Rigidbody>();
    //    endEffectorRb.constraints = RigidbodyConstraints.FreezeRotation;  // Prevent rotation of end effector
    //}



    public override void OnActionReceived(ActionBuffers actions)
    {

        if (!ispickActive)
        {
            // Check if agent1 has reached its target
            if (agent0 != null && agent0.GetComponent<task1>().task1done)
            {
                Debug.Log("pick Activated!");
                StartCoroutine(ActivateAgentWithDelay());
            }
            return;

            // Coroutine to delay activation
            IEnumerator ActivateAgentWithDelay()
            {
                yield return new WaitForSeconds(0f); // Wait for 2 seconds
                ispickActive = true;
            }// Skip further processing until activation
        }

        float arm1Action = actions.ContinuousActions[0];
        float arm2Action = actions.ContinuousActions[1];

        JointMotor arm1Motor = arm1Joint.motor;
        arm1Motor.targetVelocity = arm1Action * maxMotorVelocity;
        //arm1Motor.targetVelocity = arm1Action;
        arm1Motor.force = motorForce;
        arm1Joint.motor = arm1Motor;
        arm1Joint.useMotor = true;

        JointMotor arm2Motor = arm2Joint.motor;
        arm2Motor.targetVelocity = arm2Action * maxMotorVelocity;
        //arm2Motor.targetVelocity = arm2Action;
        arm2Motor.force = motorForce;
        arm2Joint.motor = arm2Motor;
        arm2Joint.useMotor = true;

        currentStepCount++; // Increment step counter

        if (currentStepCount >= maxStepsPerEpisode)
        {
            //Debug.Log("Reached max steps for this episode. Ending episode.");

            EndEpisode();

        }




        if (targetReached == true)
        {
            AddReward(1.0f);
            Debug.Log("Agent 2 reached Target");
            EndEpisode(); // End the episode
        }



        // Coroutine to smoothly reset the arm and then end the episod


        if (tablehit == true)
        {
            AddReward(-0.01f);

            EndEpisode();

            //Debug.Log("Episode has been ended.");
        }

        //if (IsTargetToppled())
        //{
        //AddReward(-1.0f);

        //EndEpisode();

        //}

        //if (Time.time - episodeStartTime > maxEpisodeDuration)
        //{
        //Debug.Log("Episode timed out. Ending episode.");
        //EndEpisode();
        //}

        if (ArmxTarget == true)
        {
            AddReward(-0.0001f);
            EndEpisode();

            //Debug.Log("Episode has been ended.");
        }

        if (ArmxTable == true)
        {
            EndEpisode();
            //Debug.Log("Episode has been ended.");
        }

    }

    public void EndEpisodeOnTargetReached()
    {
        //Debug.Log("End effector reached the target!");
        //AddReward(1.0f);  // Reward for reaching the target
        //EndEpisode();     // End the episode
        //Debug.Log("Episode has been ended.");
        targetReached = true;

    }

    public void EndEpisodeOnHittingTable()
    {
        //Debug.Log("End effector reached the target!");
        //AddReward(1.0f);  // Reward for reaching the target
        //EndEpisode();     // End the episode
        //Debug.Log("Episode has been ended.");
        tablehit = true;

    }

    public void ArmHitTarget()
    {
        ArmxTarget = true;
    }

    public void ArmHitTable()
    {
        ArmxTable = true;
    }



    public override void Heuristic(in ActionBuffers actionsOut)
    {
        if (!ispickActive)
        {
            Debug.Log("pick still inactive");
        }// Skip actions if agent2 is not active
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal"); // Horizontal axis
        continuousActionsOut[1] = Input.GetAxis("Vertical");   // Vertical axis

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Add joint positions and target position as observations
        sensor.AddObservation(arm1Joint.angle); // Current angle of arm1Joint
        sensor.AddObservation(arm2Joint.angle); // Current angle of arm2Joint
        sensor.AddObservation(targetPosition.position); // Target position

    }

    public override void OnEpisodeBegin()
    {
        //targetReached = false; // Reset the target reached flag
        //Debug.Log("New Episode has started.");
        //ResetArmJoints();

        //if (dummy != null)
        //{
        //    dummy.gameObject.SetActive(false); // Deactivate the dummy's GameObject
        //}

        //if (Target != null)
        //{
        //    Target.gameObject.SetActive(true); // Reactivate the target's GameObject
        //}




        Debug.Log($"Steps in the previous episode: {currentStepCount}");
        currentStepCount = 0; // Reset step counter at the start of each episode

        if (targetReached) // A flag you set when the goal is reached
        {
            // Reset arm1
            arm1.transform.position = initialArm1Position;
            arm1.transform.rotation = initialArm1Rotation;

            //Rigidbody arm1Rb = arm1.GetComponent<Rigidbody>();
            if (arm1 != null)
            {
                arm1.linearVelocity = Vector3.zero;
                arm1.angularVelocity = Vector3.zero;
                arm1.useGravity = false; // Disable gravity
                arm1.constraints = RigidbodyConstraints.FreezePositionY; // Freeze necessary axes
            }

            // Reset arm2
            arm2.transform.position = finalArm2Position;
            arm2.transform.rotation = initialArm2Rotation;

            //Rigidbody arm2Rb = arm2.GetComponent<Rigidbody>();
            if (arm2 != null)
            {
                arm2.linearVelocity = Vector3.zero;
                arm2.angularVelocity = Vector3.zero;
                arm2.useGravity = false; // Disable gravity
                arm2.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation; // Freeze necessary axes
            }

            // Reset the target
            Target.transform.position = initialTargetPosition;
            Target.transform.rotation = initialTargetRotation;

            //Rigidbody targetRb = Target.GetComponent<Rigidbody>();
            if (Target != null)
            {
                Target.linearVelocity = Vector3.zero;
                Target.angularVelocity = Vector3.zero;
                Target.useGravity = false; // Disable gravity
            }

            // Reset end-effector
            endEffector.transform.position = initialEndEffectorPosition;
            endEffector.transform.rotation = initialEndEffectorRotation;

            //Rigidbody eeRb = endEffector.GetComponent<Rigidbody>();
            if (endEffector != null)
            {
                endEffector.linearVelocity = Vector3.zero;
                endEffector.angularVelocity = Vector3.zero;
                endEffector.useGravity = false; // Disable gravity
                endEffector.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation; // Freeze necessary axes
            }

            if (dummy != null)
            {
                dummy.linearVelocity = Vector3.zero;
                dummy.angularVelocity = Vector3.zero;
                dummy.useGravity = false; // Disable gravity
                dummy.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation;
            }


            // Optionally stop the agent from further actions
            this.enabled = false; // Prevents further execution
            Debug.Log("Agent and its components are reset and gravity disabled.");
        }

        arm1.transform.position = initialArm1Position;
        arm1.transform.rotation = initialArm1Rotation;
        arm1.linearVelocity = Vector3.zero;
        arm1.angularVelocity = Vector3.zero;

        // Reset arm2
        arm2.transform.position = finalArm2Position;
        arm2.transform.rotation = initialArm2Rotation;
        arm2.linearVelocity = Vector3.zero;
        arm2.angularVelocity = Vector3.zero;
        //ResetTargetPosition();

        Target.transform.position = initialTargetPosition;
        Target.transform.rotation = initialTargetRotation;
        Target.linearVelocity = Vector3.zero;
        Target.angularVelocity = Vector3.zero;

        endEffector.transform.position = initialEndEffectorPosition;
        endEffector.transform.rotation = initialEndEffectorRotation;
        endEffector.linearVelocity = Vector3.zero;
        endEffector.angularVelocity = Vector3.zero;

        targetReached = false;
        tablehit = false;
        ArmxTarget = false;
        ArmxTable = false;

    }

    //private void ResetArmJoints()
    //{
    //    JointMotor motor = arm1Joint.motor;
    //    motor.targetVelocity = 0.001f;  // Ensure the joint stays still
    //    motor.force = 50f;
    //    arm1Joint.motor = motor;
    //    arm1Joint.useMotor = true;

    //    JointMotor motor2 = arm2Joint.motor;
    //    motor2.targetVelocity = 0.001f;  // Ensure the joint stays still
    //    motor2.force = 50f;
    //    arm2Joint.motor = motor2;
    //    arm2Joint.useMotor = true;
    //}

    private bool IsTargetToppled()
    {
        // Calculate the angle between the target's "up" direction and the world "up" direction
        float angle = Vector3.Angle(Target.transform.up, Vector3.up);

        // Check if the angle exceeds the threshold
        return angle > toppleAngleThreshold;
    }

    //private void SetJointMotor(HingeJoint joint, float targetAngle)
    //{
    //    var motor = joint.motor;
    //    motor.targetVelocity = (targetAngle - joint.angle) * 10f;
    //    motor.force = 10f;
    //    motor.freeSpin = false;
    //    joint.motor = motor;
    //    joint.useMotor = true;
    //}

    //private void ResetTargetPosition()
    //{
    //    // Reset the target position (you can randomize this if desired)
    //    targetPosition.position = new Vector3(
    //        2f,// Randomize x
    //        0.4f,//andomize y
    //        0.7f//andomize z
    //    );

    //    //Debug.Log($"Target position reset to: {targetPosition.position}");
    //}


    //private bool TouchesTarget()
    //{
    //    // Get the bounds of the base's collider
    //    Collider endEffectorCollider = endEffector.GetComponent<Collider>();
    //    //if (baseCollider == null)
    //    //{
    //    //    Debug.LogError("Base does not have a Collider attached!");
    //    //    return false;
    //    //}

    //    // Array of all obstacles
    //    Collider targetCollider = Target.GetComponent<Collider>();



    //    // Check if the base's collider intersects with the obstacle's collider
    //    if (endEffectorCollider.bounds.Intersects(targetCollider.bounds))
    //    {
    //        return true; // Collision detected
    //    }


    //    return false; // No collisions
    //}


}
