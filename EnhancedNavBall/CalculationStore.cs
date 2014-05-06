using UnityEngine;
using KSP;

public class CalculationStore
{
    public Vector3d RadialPlus;
    public Vector3d ProgradeOrbit;
    public Vector3d ProgradeSurface;
    public Vector3d NormalPlus;
    public Vector3d ManeuverPlus = Vector3d.zero;

    //kujuman
    public Vector3d ThrustPlus;

    public bool ManeuverPresent;

    public void RunCalculations(
        Vessel vessel,
        Quaternion gymbal)
    {
        // Calculations thanks to Mechjeb
        Vector3d CoM = vessel.findWorldCenterOfMass();
        Vector3d up = (CoM - vessel.mainBody.position).normalized;
        Vector3d velocityVesselOrbit = vessel.orbit.GetVel();
        Vector3d velocityVesselOrbitUnit = velocityVesselOrbit.normalized;
        Vector3d radialPlus = Vector3d.Exclude(velocityVesselOrbit, up).normalized;
        Vector3d velocityVesselSurface = velocityVesselOrbit - vessel.mainBody.getRFrmVel(CoM);
        Vector3d velocityVesselSurfaceUnit = velocityVesselSurface.normalized;

        RadialPlus = gymbal * radialPlus;
        NormalPlus = gymbal * -Vector3d.Cross(radialPlus, velocityVesselOrbitUnit);
        ProgradeOrbit = gymbal * velocityVesselOrbitUnit;
        ProgradeSurface = gymbal * velocityVesselSurfaceUnit;



        //kujuman
        Vector3 dirInitial = avgThrustDirection(vessel).normalized;
        dirInitial = (Vector3d)vessel.ReferenceTransform.InverseTransformDirection(dirInitial);
        //Direction of vessel-wide thrust vector is now in the "Control From Here" reference frame
        //Do some jiggling to align axes with navball
        Vector3 dir = dirInitial;
        dir.x = -1 * dirInitial.x;
        dir.y = dirInitial.z;
        dir.z = -1 * dirInitial.y;
        ThrustPlus = dir;




        if (vessel.patchedConicSolver.maneuverNodes.Count > 0)
        {
            Vector3d burnVector = vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(vessel.orbit);
            ManeuverPlus = gymbal * burnVector.normalized;
            ManeuverPresent = true;
        }
        else
        {
            ManeuverPresent = false;
        }
    }


    //kujuman
    /// <summary>
    /// This method returns a [reference frame] vector of the average of all thrust from active ModuleEngines and ModuleEnginesFX. Magnitude is net thrust in kN.
    /// Assumes no part contains more than one total ModuleEngines or ModuleEnginesFX
    /// </summary>
    /// <param name="v">the vessel</param>
    /// <returns></returns>
    public Vector3d avgThrustDirection(Vessel v)
    {
        Vector3 dir = new Vector3();

        ModuleEngines me;
        ModuleEnginesFX mefx;

        Vector3 engineAvgTransform;

        foreach (Part p in v.parts)
        {
            engineAvgTransform = new Vector3();
            me = new ModuleEngines();

            if (p.Modules.Contains("ModuleEngines"))
            {
                me = p.GetComponent<ModuleEngines>();

                foreach (Transform t in me.thrustTransforms)
                {
                    engineAvgTransform += t.forward;
                }
                engineAvgTransform.Normalize();
                engineAvgTransform = engineAvgTransform * me.finalThrust;

                goto AddThrustComponent;
            }

            if (p.Modules.Contains("ModuleEnginesFX"))
            {
                mefx = p.GetComponent<ModuleEnginesFX>();

                foreach (Transform t in mefx.thrustTransforms)
                {
                    engineAvgTransform += t.forward;
                }

                engineAvgTransform.Normalize();
                engineAvgTransform = engineAvgTransform * mefx.finalThrust;

                goto AddThrustComponent;
            }
            else
                goto NextPart;

        AddThrustComponent:
            dir += engineAvgTransform;

    NextPart:
        ;
        }

        return dir;
    }
    /// <summary>
    /// Use to print formatted Debug.Log messeges.
    /// </summary>
    /// <param name="engName">Some identifier</param>
    /// <param name="thrust">thrust in kN</param>
    /// <param name="dir">Direction of thrust</param>
    private void pEI(string engName , float thrust, Vector3 dir)
    {
        Debug.Log("=========" + engName + "=========");
        Debug.Log("Thrust = " + thrust + " kN");
        Debug.Log(dir.ToString());
    }


}