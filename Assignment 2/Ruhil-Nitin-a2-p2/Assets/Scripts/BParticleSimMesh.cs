using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Check this out we can require components be on a game object!
[RequireComponent(typeof(MeshFilter))]

public class BParticleSimMesh : MonoBehaviour
{
    public struct BSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring
        public int attachedParticle;            // index of the attached other particle (use me wisely to avoid doubling springs and sprign calculations)
    }

    public struct BContactSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring
        public Vector3 attachPoint;             // the attached point on the contact surface
    }

    public struct BParticle
    {
        public Vector3 position;                // position information
        public Vector3 velocity;                // velocity information
        public float mass;                      // mass information
        public BContactSpring contactSpring;    // Special spring for contact forces
        public bool attachedToContact;          // to check if particle currently attached to a contact (ground plane contact)
        public List<BSpring> attachedSprings;   // all attached springs, as a list in case we want to modify later fast
        public Vector3 currentForces;           // accumulate forces here on each step        
    }

    public struct BPlane
    {
        public Vector3 position;                // plane position
        public Vector3 normal;                  // plane normal
    }

    public float contactSpringKS = 1000.0f;     // contact spring coefficient with default 1000
    public float contactSpringKD = 20.0f;       // contact spring daming coefficient with default 20

    public float defaultSpringKS = 100.0f;      // default spring coefficient with default 100
    public float defaultSpringKD = 1.0f;        // default spring daming coefficient with default 1

    public bool debugRender = false;            // To render or not to render


    /*** 
     * I've given you all of the above to get you started
     * Here you need to publicly provide the:
     * - the ground plane transform (Transform)
     * - handlePlaneCollisions flag (bool)
     * - particle mass (float)
     * - useGravity flag (bool)
     * - gravity value (Vector3)
     * Here you need to privately provide the:
     * - Mesh (Mesh)
     * - array of particles (BParticle[])
     * - the plane (BPlane)
     ***/

    [Header("Simulation Controls")]
    public Transform groundPlaneTransform;      // plane transform from the scene
    public bool handlePlaneCollisions = true;   // turn ground contact on/off
    public float particleMass = 1.0f;           // mass of each particle
    public bool useGravity = true;              // use gravity?
    public Vector3 gravity = new Vector3(0, -9.8f, 0);

    Mesh mesh;                                  // mesh of this object
    BParticle[] particles;                      // all particles (one per vertex)
    BPlane plane;                               // ground plane
    Vector3[] originalVertices;                 // initial local-space mesh vertices
    Vector3[] workingVertices;                  // mutable local-space vertices

    /// <summary>
    /// Init everything
    /// HINT: in particular you should probbaly handle the mesh, init all the particles, and the ground plane
    /// HINT 2: I'd for organization sake put the init particles and plane stuff in respective functions
    /// HINT 3: Note that mesh vertices when accessed from the mesh filter are in local coordinates.
    ///         This script will be on the object with the mesh filter, so you can use the functions
    ///         transform.TransformPoint and transform.InverseTransformPoint accordingly 
    ///         (you need to operate on world coordinates, and render in local)
    /// HINT 4: the idea here is to make a mathematical particle object for each vertex in the mesh, then connect
    ///         each particle to every other particle. Be careful not to double your springs! There is a simple
    ///         inner loop approach you can do such that you attached exactly one spring to each particle pair
    ///         on initialization. Then when updating you need to remember a particular trick about the spring forces
    ///         generated between particles. 
    /// </summary>
    void Start()
    {
        mesh = GetComponent<MeshFilter>().mesh;
        originalVertices = mesh.vertices;
        workingVertices = new Vector3[originalVertices.Length];

        InitPlane();
        InitParticles();

    }


    /*** BIG HINT: My solution code has as least the following functions
     * InitParticles()
     * InitPlane()
     * UpdateMesh() (remember the hint above regarding global and local coords)
     * ResetParticleForces()
     * ...
     ***/
    void InitParticles()
    {
        int count = originalVertices.Length;
        particles = new BParticle[count];

        // 1) Create particles at each vertex
        for (int i = 0; i < count; i++)
        {
            Vector3 worldPos = transform.TransformPoint(originalVertices[i]);

            BParticle p = new BParticle();
            p.position = worldPos;
            p.velocity = Vector3.zero;
            p.mass = particleMass;
            p.currentForces = Vector3.zero;
            p.attachedToContact = false;
            p.contactSpring = new BContactSpring();
            p.attachedSprings = new List<BSpring>();

            particles[i] = p;
        }

        // 2) Create springs between every unique pair (i < j)
        for (int i = 0; i < count; i++)
        {
            for (int j = i + 1; j < count; j++)
            {
                float restLen = Vector3.Distance(particles[i].position, particles[j].position);

                BSpring s = new BSpring();
                s.ks = defaultSpringKS;
                s.kd = defaultSpringKD;
                s.restLength = restLen;
                s.attachedParticle = j;

                // Attach spring only to particle i (avoid duplicates).
                particles[i].attachedSprings.Add(s);
            }
        }
    }

    // Initialize the ground plane
    void InitPlane()
    {
        if (groundPlaneTransform != null)
        {
            plane.position = groundPlaneTransform.position;
            plane.normal = groundPlaneTransform.up.normalized;
        }
        else
        {
            // default: plane y=0, normal up
            plane.position = Vector3.zero;
            plane.normal = Vector3.up;
        }
    }

    // Reset forces on each particle
    void ResetParticleForces()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            particles[i].currentForces = Vector3.zero;
        }
    }

    /// <summary>
    /// Draw a frame with some helper debug render code
    /// </summary>
    public void Update()
    {
        float dt = Time.deltaTime;
        if (particles == null || particles.Length == 0)
            return;

        Simulate(dt);

        // This will work if you have a correctly made particles array
        if (debugRender)
        {
            int particleCount = particles.Length;
            for (int i = 0; i < particleCount; i++)
            {
                Debug.DrawLine(particles[i].position, particles[i].position + particles[i].currentForces, Color.blue);

                int springCount = particles[i].attachedSprings.Count;
                for (int j = 0; j < springCount; j++)
                {
                    Debug.DrawLine(particles[i].position, particles[particles[i].attachedSprings[j].attachedParticle].position, Color.red);
                }
            }
        }

    }
    // The main simulation step
    void Simulate(float dt)
    {
        ResetParticleForces();

        // 1) Gravity
        if (useGravity)
        {
            Vector3 gForce = gravity * particleMass;
            for (int i = 0; i < particles.Length; i++)
            {
                particles[i].currentForces += gForce;
            }
        }

        // 2) Ground contact penalty springs
        if (handlePlaneCollisions)
        {
            HandleGroundContacts();
        }

        // 3) Internal particle-particle springs
        ApplySpringForces();

        // 4) Integrate (Symplectic Euler)
        Integrate(dt);

        // 5) Update mesh from particles
        UpdateMesh();
    }

    // Handle contact springs with the ground plane
    void HandleGroundContacts()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            BParticle p = particles[i];

            // signed distance from plane: (x - x0)·n
            float signedDistance = Vector3.Dot(p.position - plane.position, plane.normal);

            // detect new penetration
            if (signedDistance < 0f && !p.attachedToContact)
            {
                p.attachedToContact = true;
                p.contactSpring.ks = contactSpringKS;
                p.contactSpring.kd = contactSpringKD;
                p.contactSpring.restLength = 0f;

                // attachPoint is nearest point on plane
                Vector3 projection = p.position - signedDistance * plane.normal;
                p.contactSpring.attachPoint = projection;
            }

            // if attached, apply penalty force
            if (p.attachedToContact)
            {
                float penetration = Vector3.Dot(p.position - p.contactSpring.attachPoint, plane.normal);

                // if we are above or exactly at plane (no more penetration), detach
                if (penetration >= 0f)
                {
                    p.attachedToContact = false;
                }
                else
                {
                    // F = -ks * ((xp - xg)·n) n - kd * vp
                    Vector3 Fspring = -p.contactSpring.ks * penetration * plane.normal;
                    Vector3 Fdamp = -p.contactSpring.kd * p.velocity;
                    p.currentForces += Fspring + Fdamp;
                }
            }
            particles[i] = p; // write back modification
        }
    }

    // Apply particle-particle spring forces (reflected force trick)
    void ApplySpringForces()
    {
        int count = particles.Length;

        for (int i = 0; i < count; i++)
        {
            BParticle pi = particles[i];
            int springCount = pi.attachedSprings.Count;

            for (int sIndex = 0; sIndex < springCount; sIndex++)
            {
                BSpring s = pi.attachedSprings[sIndex];
                int j = s.attachedParticle;
                BParticle pj = particles[j];

                Vector3 xi = pi.position;
                Vector3 xj = pj.position;
                Vector3 vi = pi.velocity;
                Vector3 vj = pj.velocity;

                Vector3 dir = xi - xj;
                float dist = dir.magnitude;
                if (dist < 1e-6f)
                    continue;

                Vector3 n = dir / dist; // unit vector

                // spring force: ks (l - |xi - xj|) * n
                float stretch = s.restLength - dist;
                Vector3 Fspring = s.ks * stretch * n;

                // damping term along n: -kd ((vi - vj)·n) n
                float relVelAlong = Vector3.Dot(vi - vj, n);
                Vector3 Fdamp = -s.kd * relVelAlong * n;

                Vector3 F = Fspring + Fdamp;

                // apply equal and opposite forces
                pi.currentForces += F;
                pj.currentForces -= F;

                particles[j] = pj;
            }
            particles[i] = pi;
        }
    }

    // Symplectic Euler integration
    void Integrate(float dt)
    {
        for (int i = 0; i < particles.Length; i++)
        {
            BParticle p = particles[i];

            Vector3 acceleration = p.currentForces / p.mass;
            p.velocity += acceleration * dt;       // v_{t+1} = v_t + a dt
            p.position += p.velocity * dt;        // x_{t+1} = x_t + v_{t+1} dt

            particles[i] = p;
        }
    }

    // Write particle positions back into the mesh (world -> local)
    void UpdateMesh()
    {
        for (int i = 0; i < particles.Length; i++)
        {
            workingVertices[i] = transform.InverseTransformPoint(particles[i].position);
        }

        mesh.vertices = workingVertices;
        mesh.RecalculateBounds();
        mesh.RecalculateNormals();
    }
}