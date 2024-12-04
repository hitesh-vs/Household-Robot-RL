using UnityEngine;

public class eeCollider : MonoBehaviour
{
    public PicknPlace agentScript; // Reference to the PicknPlace agent script
    public Transform endEffector; // Reference to the end effector
    //public Attach attachScript;  // Reference to the Attach script
    private bool isPickedUp = false;  // To track if the object is picked up
    public GameObject dummyObject;
    //private GameObject pickedTarget = null;

    //void Start()
    //{
    //    if (dummyObject != null)
    //    {
    //        dummyObject.SetActive(false); // Ensure the dummy is disabled initially
    //    }
    //}

    void OnCollisionEnter(Collision collision)
    {
        //isPickedUp = true;

        // Disable the target object
        //collision.gameObject.SetActive(false);

        //Enable the dummy object
        if (dummyObject != null)
        {
            dummyObject.SetActive(true);
        }

        // Check if the collided object is the target
        if (collision.gameObject.CompareTag("Target"))
        {
            //// Parent the target to the end effector
            //isPickedUp = true;
            ////collision.gameObject.transform.SetParent(endEffector); // Attach target to end effector
            ////collision.gameObject.GetComponent<Rigidbody>().isKinematic = true; // Disable target's physics
            //Rigidbody targetRigidbody = collision.gameObject.GetComponent<Rigidbody>();
            //if (targetRigidbody != null)
            //{
            //    HingeJoint joint = collision.gameObject.AddComponent<HingeJoint>();
            //    joint.connectedBody = endEffector.GetComponent<Rigidbody>();

            //    // Optional: Adjust joint settings for stability
            //    joint.breakForce = Mathf.Infinity;
            //    joint.breakTorque = Mathf.Infinity;

            //    Debug.Log("Target attached using Fixed Joint");
            //}

            //if (attachScript != null)
            //{
            //    attachScript.AttachToEndEffector(endEffector);  // Set the flag 'att' to true
            //}

            //collision.transform.SetParent(endEffector);  // Set the target as a child of the end effector
            //Rigidbody targetRigidbody = collision.gameObject.GetComponent<Rigidbody>();
            //if (targetRigidbody != null)
            //{
            //    targetRigidbody.isKinematic = true;  // Disable physics on the target
            //}

            // Notify the agent script that the target has been picked up
            collision.gameObject.SetActive(false);
            if (agentScript != null)
            {
                agentScript.EndEpisodeOnTargetReached(); // Keep your existing logic for ending the episode
            }
        }

        // Check if the collided object is the table
        if (collision.gameObject.CompareTag("Table"))
        {
            // Notify the agent script that the table was hit
            if (agentScript != null)
            {
                agentScript.EndEpisodeOnHittingTable(); // Existing logic for handling table collision
            }
        }
    }

    //public void ReleaseObject()
    //{
    //    // Detach the target when releasing it
    //    if (isPickedUp)
    //    {
    //        Transform target = endEffector.GetChild(0); // Assuming the target is the only child
    //        target.SetParent(null); // Detach the target
    //        target.GetComponent<Rigidbody>().isKinematic = false; // Re-enable target's physics
    //        isPickedUp = false; // Reset pickup state
    //    }
    //}
}
