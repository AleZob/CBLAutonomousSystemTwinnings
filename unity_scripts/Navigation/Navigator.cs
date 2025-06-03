using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor.EditorTools;
using UnityEditor;

[CustomEditor(typeof(Navigator))]
public class NavigatorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        Navigator nav = (Navigator)target;
        if (GUILayout.Button("Resend goal pose"))
        {
            nav.navigateToTarget();
        }
    }
}

[RequireComponent(typeof(MovementGoal))]
public class Navigator : MonoBehaviour
{
    // First element must be the depot.
    [SerializeField] private List<BusStop> stops = new List<BusStop>();
    [SerializeField] private float touchDistance = 0.5f;
    [SerializeField] private float stopDelay = 1.0f;
    [SerializeField] private int targetIndex = 0;

    [SerializeField] private int demandTreshold;

    private MovementGoal movementGoal;
    private float timeStopped = 0.0f;
    private int lastIndex;

    private BusStop target { get => stops[targetIndex]; }
    private float distance { get => movementGoal.DistanceToObject(target.gameObject); }
    private bool atDepot { get => arrived && targetIndex == 0; }

    [SerializeField] private bool arrived = false;

    private void Start()
    {
        if (touchDistance <= 0)
        {
            Debug.LogWarning("The touch distance should be positive.");
        }
        movementGoal = GetComponent<MovementGoal>();
        lastIndex = stops.Count - 1;
        navigateToTarget();
    }

    private void Update()
    {
        if (targetIndex == 0 && distance <= touchDistance)
        {
            // We are finished at the depot.
            for(int i = 1; i < stops.Count; i++)
            {
                BusStop stop = stops[i];
                if (stop.demand >= demandTreshold)
                {
                    // We found a busy stop.
                    targetIndex = i;
                    navigateToTarget();
                    break;
                }
            }
        }
        else if (!arrived && distance <= touchDistance)
        {
            // We just arrived.
            arrived = true;
            onArrival();
        }
        else if (!arrived)
        {
            // We are still underway.
        }
        // // Commented out to fix bug.
        // else if (arrived && distance > touchDistance)
        // {
        //     // We left early.
        //     arrived = false;
        // }
        else
        {
            // We are waiting at a stop.
            timeStopped += Time.deltaTime;
            if (timeStopped >= stopDelay)
            {
                arrived = false;
                timeStopped = 0.0f;
                navigateToTarget();
            }
        }
    }

    internal void navigateToTarget()
    {
        movementGoal.NavigateToObject(target.gameObject);
        print($"Navigating towards target number {targetIndex}.");
    }

    private void onArrival()
    {
        timeStopped = 0.0f;
        target.demand = 0;
        if (targetIndex == 0)
        {
            // We arrived at depot. Don't wait.
            timeStopped = stopDelay;
        }
        else if (targetIndex < lastIndex)
        {
            // Time to go to the next stop
            targetIndex++;

        }
        else if (targetIndex == lastIndex)
        {
            // Done, time to go to target
            targetIndex = 0;
        }
        else
        {
            Debug.LogError("Unreachable state");
        }
    }
}
