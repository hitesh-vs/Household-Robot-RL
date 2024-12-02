using UnityEngine;

public class armcollider : MonoBehaviour
{
    public PicknPlace agentScript;  // Reference to the PicknPlace agent script

    void OnCollisionEnter(Collision collision)
    {


        // Check if the collided object is the target
        if (collision.gameObject.CompareTag("Target"))
        {
            // Notify the agent script that the target has been reached
            if (agentScript != null)
            {
                agentScript.ArmHitTarget();
            }
        }

        if (collision.gameObject.CompareTag("Table"))
        {
            // Notify the agent script that the target has been reached
            if (agentScript != null)
            {
                agentScript.ArmHitTable();
            }
        }
    }
}
