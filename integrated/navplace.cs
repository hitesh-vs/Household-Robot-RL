using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.MLAgents;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class navplace : Agent
{
    public Rigidbody mobileBase;
    public Transform targetPosition;
    public Rigidbody NavTarget;
    public Rigidbody obs1;
    public Rigidbody obs2;
    public Rigidbody obs3;
    public Rigidbody obs4;
    public Rigidbody table;
    public PicknPlace agent1;

    private float previousDistanceToTarget;
    private bool isActive = false;
    public int maxSteps = 500000;

    // Define the bounding box size around the target
    private float boundingBoxSizeX = 5.5f;
    private float boundingBoxSizeZ = 6f;

    //Timekeeping
    private float episodeStartTime; // Time when the episode started
    public float timeLimit = 120f;




    public override void OnEpisodeBegin()
    {
        // Reset Rigidbody velocities
        mobileBase.linearVelocity = Vector3.zero;
        mobileBase.angularVelocity = Vector3.zero;

        // Find a safe spawn position
        Vector3 spawnPosition;
        spawnPosition = new Vector3(2f, 0.05f, 0.05f);

        // Set position and reset orientation
        mobileBase.transform.localPosition = spawnPosition;
        mobileBase.transform.rotation = Quaternion.Euler(0, 0, 0);

        // Previous distance
        previousDistanceToTarget = Vector3.Distance(mobileBase.transform.localPosition, targetPosition.localPosition);

        // Randomize target position within a 5x5 area
        targetPosition.localPosition = new Vector3(Random.Range(-3f, -1f), 0.1f, Random.Range(-3f, 3f));
        //obs 
        //obs1.transform.localPosition = new Vector3(1f, 0.1f, 2f);
        //obs2.transform.localPosition = new Vector3(1f, 0.1f, -2f);
        //obs3.transform.localPosition = new Vector3(-0.6f, 0.1f, 1f);
        //obs4.transform.localPosition = new Vector3(-0.6f, 0.1f, -1f);

        // Randomize obstacle positions
        List<Vector3> occupiedPositions = new List<Vector3> { targetPosition.localPosition };
        RandomizeObstaclePosition(obs1, occupiedPositions);
        RandomizeObstaclePosition(obs2, occupiedPositions);
        RandomizeObstaclePosition(obs3, occupiedPositions);
        RandomizeObstaclePosition(obs4, occupiedPositions);

        // Track the previous distance
        previousDistanceToTarget = Vector3.Distance(mobileBase.transform.localPosition, targetPosition.localPosition);

        episodeStartTime = Time.time;

    }

    private void RandomizeObstaclePosition(Rigidbody obstacle, List<Vector3> occupiedPositions)
    {
        Vector3 newPosition;
        bool isValid;

        do
        {
            // Generate a random position within the range
            newPosition = new Vector3(
                Random.Range(-4f, 1f),
                0.1f,
                Random.Range(-4f, 4f)
            );

            // Check if the position overlaps with any existing positions
            isValid = true;
            foreach (Vector3 occupied in occupiedPositions)
            {
                if (Vector3.Distance(newPosition, occupied) < 0.5f) // Adjust the threshold as needed
                {
                    isValid = false;
                    break;
                }
            }
        }
        while (!isValid);

        // Assign the valid position and add it to the list of occupied positions
        obstacle.transform.localPosition = newPosition;
        occupiedPositions.Add(newPosition);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float baseX = mobileBase.transform.localPosition.x;
        //float baseY = mobileBase.transform.localPosition.y;
        float baseZ = mobileBase.transform.localPosition.z;
        float targetX = targetPosition.localPosition.x;
        float targetY = targetPosition.localPosition.y;
        float targetZ = targetPosition.localPosition.z;

        // Add observations
        sensor.AddObservation(baseX);
        //sensor.AddObservation(baseY);
        sensor.AddObservation(baseZ);
        sensor.AddObservation(targetX);
        //sensor.AddObservation(targetY);
        sensor.AddObservation(targetZ);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //if (!isActive)
        //{
        //    Debug.Log("nav still inactive");
        //}// Skip actions if agent2 is not active

        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Arm1");  // Control base movement (x)
        continuousActionsOut[1] = Input.GetAxis("Arm2");    // Control base movement (z)
        //continuousActionsOut[2] = Input.GetAxis("Arm1");        // Control arm1 joint angle
        //continuousActionsOut[3] = Input.GetAxis("Arm2");        // Control arm2 joint angle
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {

        //if (!isActive)
        //{
        //    // Check if agent1 has reached its target
        //    if (agent1 != null && agent1.GetComponent<PicknPlace>().targetReached)
        //    {
        //        Debug.Log("Agent2 Activated!");
        //        StartCoroutine(ActivateAgentWithDelay());
        //    }
        //    return;

        //    // Coroutine to delay activation
        //    IEnumerator ActivateAgentWithDelay()
        //    {
        //        yield return new WaitForSeconds(5); // Wait for 2 seconds
        //        isActive = true;
        //    }// Skip further processing until activation
        //}

        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actionBuffers.ContinuousActions[0];
        controlSignal.z = actionBuffers.ContinuousActions[1];

        // Move the agent based on the action
        float MoveSpeed = 10f;
        mobileBase.position += controlSignal * Time.deltaTime * MoveSpeed;

        // Reward calculations based on distance to target
        float currentDistanceToTarget = Vector3.Distance(mobileBase.transform.localPosition, targetPosition.localPosition);

        if (TouchesTar()) // Goal reached
        {
            SetReward(1.0f); // High reward for reaching the target
            EndEpisode();
            Debug.Log($"dusra");
        }

        if (TouchesObs()) // Goal reached
        {
            SetReward(-1.0f); // High reward for reaching the target
            EndEpisode();
            Debug.Log($"hitesh is great");
        }
        //else
        //{
        float rewardForProgress = (previousDistanceToTarget - currentDistanceToTarget);
        if (rewardForProgress >= 0.1)
        {

            SetReward(0.01f); // Reward for progress
        }
        //}

        //Penalize for going outside the bounding box
        if (IsOutsideBoundingBox())
        {
            AddReward(-1.0f); // Negative reward for leaving the allowed area
            EndEpisode(); // End episode if agent leaves the bounding box
            Debug.Log($"pehela nasha ");
        }

        //if (mobileBase.transform.localPosition.y < 0.0f)
        //{
        //    //Debug.Log("Eps end due to base fall");
        //    SetReward(-1.0f);
        //    EndEpisode();

        //}

        previousDistanceToTarget = currentDistanceToTarget;

        float timeElapsed = Time.time - episodeStartTime;

        // Check if the time limit has been exceeded
        if (timeElapsed >= timeLimit)
        {
            // Apply a penalty and end the episode if time limit exceeded
            AddReward(-1.0f);  // Negative reward for exceeding time limit
            EndEpisode();      // End the episode
            Debug.Log($"habibi");
        }
    }



    private bool TouchesTar()
    {
        Collider MobBaseCol = mobileBase.GetComponent<Collider>();
        Collider NavTarCol = NavTarget.GetComponent<Collider>();

        // Check if the base's collider intersects with the target's collider
        if (MobBaseCol.bounds.Intersects(NavTarCol.bounds))
        {
            return true; // Collision detected
        }

        return false; // No collision
    }

    private bool TouchesObs()
    {
        Collider mobBaseCol = mobileBase.GetComponent<Collider>();
        GameObject[] obstacles = GameObject.FindGameObjectsWithTag("Obstacle");
        Collider TableCol = table.GetComponent<Collider>();

        foreach (GameObject obs in obstacles)
        {
            Collider obsCol = obs.GetComponent<Collider>();
            if (mobBaseCol.bounds.Intersects(obsCol.bounds))
            {
                return true; // Collision detected with at least one obstacle
            }
        }

        if (mobBaseCol.bounds.Intersects(TableCol.bounds))
        {
            return true; // Collision detected with at least one obstacle
        }


        return false; // No collisions detected
    }


    private bool IsOutsideBoundingBox()
    {
        // Get the position of the agent relative to the target
        Vector3 agentPosition = mobileBase.transform.localPosition;

        // Check if the agent is outside the allowed bounding box
        if (Mathf.Abs(agentPosition.x - targetPosition.localPosition.x) > boundingBoxSizeX ||
            Mathf.Abs(agentPosition.z - targetPosition.localPosition.z) > boundingBoxSizeZ)
        {
            return true; // The agent is outside the bounding box
        }

        return false; // The agent is within the bounding box
    }
}
