using UnityEngine;
using KSP;

public class CalculationStore
{
    public Vector3d RadialPlus;
    public Vector3d ProgradeOrbit;
    public Vector3d ProgradeSurface;
    public Vector3d NormalPlus;
    public Vector3d ManeuverPlus = Vector3d.zero;

    ////////////////////////////////////////////////////////////////////////
    public Vector3d ThrustPlus;
    CenterOfThrustQuery CoTQ;

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

        ////////////////////////////////////////////////////////////////////////
        Debug.Log("Before CoTQ");
        CoTQ = new CenterOfThrustQuery();
        ModuleEngines me;
        ModuleEnginesFX mefx;
        foreach (Part p in vessel.parts)
        {
            if (p.Modules.Contains("ModuleEngines"))
            {
                me = p.GetComponent<ModuleEngines>();
                me.OnCenterOfThrustQuery(CoTQ);
            }

            if (p.Modules.Contains("ModuleEnginesFX"))
            {
                mefx = p.GetComponent<ModuleEnginesFX>();
                mefx.OnCenterOfThrustQuery(CoTQ);
            }
        }
        //Debug.Log("CoTQ 2");
        //Debug.Log("CoTQ.thrust = " + CoTQ.thrust.ToString("F2"));
        //Debug.Log("CoTQ.dir = " + CoTQ.dir.ToString());
        //Debug.Log("CoTQ.pos = " + CoTQ.pos.ToString());
        //Debug.Log("Post CoTQ");
        //CoTQ.dir = (Vector3d)vessel.transform.InverseTransformDirection((Vector3d)CoTQ.dir); //uses root part transform
        CoTQ.dir = (Vector3d)vessel.ReferenceTransform.InverseTransformDirection((Vector3d)CoTQ.dir);
        CoTQ.dir.y *= -1;
        Vector3 dir = CoTQ.dir;
        dir.x = -1 * dir.x;
        dir.y = CoTQ.dir.z;
        dir.z = CoTQ.dir.y;
        Debug.Log(CoTQ.dir.ToString("F4"));
        Debug.Log(dir.ToString("F4"));
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
}