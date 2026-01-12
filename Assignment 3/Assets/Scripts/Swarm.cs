
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class Swarm : MonoBehaviour
{
    public struct BBoid
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 velocity;
        public Vector3 alignment;
        public Vector3 cohesion;
        public Vector3 separation;
        public Vector3 obstacle;
        public Vector3 currentTotalForce;
    }

    public Transform boidPrefab;

    public int numberOfBoids = 200;

    // α in the handout
    public float boidForceScale = 20f;

    public float maxSpeed = 5.0f;
    public float rotationSpeed = 40.0f;

    public float obstacleCheckRadius = 1.0f;

    public float separationWeight = 1.1f;
    public float alignmentWeight = 0.5f;
    public float cohesionWeight = 1f;
    public float goalWeight = 1f;
    public float obstacleWeight = 0.9f;
    public float wanderWeight = 0.3f;

    public float neighbourDistance = 2.0f;

    public float initializationRadius = 1.0f;
    public float initializationForwardRandomRange = 50f;

    public float goalForceScale = 10f;

    private BBoid[] boids;
    private Transform[] boidObjects;

    private float sqrNeighbourDistance;

    private Vector3 boidZeroGoal;
    private NavMeshPath boidZeroPath;
    private int currentCorner;
    private bool boidZeroNavigatingTowardGoal = false;

    // world bounds treated as obstacles for boids
    private const float worldMinX = -8f;
    private const float worldMaxX = 8f;
    private const float worldMinZ = -8f;
    private const float worldMaxZ = 8f;
    private const float worldMinY = 1f;
    private const float worldMaxY = 4f;

    private void Start()
    {
        InitBoids();
    }

    /// <summary>
    /// Initialize the array of boids.
    /// </summary>
    private void InitBoids()
    {
        boids = new BBoid[numberOfBoids];
        boidObjects = new Transform[numberOfBoids];

        sqrNeighbourDistance = neighbourDistance * neighbourDistance;

        boidZeroPath = new NavMeshPath();
        boidZeroNavigatingTowardGoal = false;
        currentCorner = 0;

        // Spawn all boids
        for (int i = 0; i < numberOfBoids; i++)
        {
            BBoid b = new BBoid();

            // Random position in a flat disc
            Vector3 pos = transform.position + Random.insideUnitSphere * initializationRadius;
            pos.y = 1.5f;
            b.position = pos;

            float yaw = Random.Range(-initializationForwardRandomRange, initializationForwardRandomRange);
            Quaternion q = Quaternion.Euler(0f, yaw, 0f);
            Vector3 fwd = (q * Vector3.forward).normalized;
            b.forward = fwd;

            // Initial velocity
            b.velocity = fwd * (maxSpeed * 0.5f);

            // Rule vectors & force
            b.alignment = Vector3.zero;
            b.cohesion = Vector3.zero;
            b.separation = Vector3.zero;
            b.obstacle = Vector3.zero;
            b.currentTotalForce = Vector3.zero;

            boids[i] = b;

            // Visual object
            if (boidPrefab != null)
            {
                Transform obj = Instantiate(boidPrefab, b.position, Quaternion.LookRotation(fwd));
                boidObjects[i] = obj;
            }
        }

        // Place boid zero on navmesh
        BBoid b0 = boids[0];
        NavMeshHit nh;
        if (NavMesh.SamplePosition(new Vector3(0f, 2f, 0f), out nh, 10f, NavMesh.AllAreas))
        {
            b0.position = nh.position;
            boids[0] = b0;

            if (boidObjects[0] != null)
                boidObjects[0].position = b0.position;
        }
    }

    /// <summary>
    /// Reset the particle forces
    /// </summary>
    public void ResetBoidForces()
    {
        if (boids == null)
        {
             return;
        }

        for (int i = 0; i < boids.Length; i++)
        {
            BBoid b = boids[i];
            b.alignment = Vector3.zero;
            b.cohesion = Vector3.zero;
            b.separation = Vector3.zero;
            b.obstacle = Vector3.zero;
            b.currentTotalForce = Vector3.zero;
            boids[i] = b;
        }
    }

    /// <summary>
    /// Sim Loop
    /// </summary>
    private void FixedUpdate()
    {
        if (boids == null || boids.Length == 0)
            return;

        float dt = Time.fixedDeltaTime;

        // Reset rule forces
        ResetBoidForces();

        int boidCount = boids.Length;

        // Apply neighbour rules and obstacle/world rules
        for (int i = 0; i < boidCount; i++)
        {
            ApplyNeighbourRules(i);
            ApplyObstacleAndWorldRules(i);
        }

        // Special goal-following behaviour for boid zero
        ApplyBoidZeroGoalRule();

        // Symplectic Euler integration and updating visual objects
        for (int i = 0; i < boidCount; i++)
        {
            BBoid b = boids[i];

            Vector3 acceleration = b.currentTotalForce;

            // v_{t+1} = v_t + a_t * dt
            b.velocity += acceleration * dt;

            // clamp speed
            if (b.velocity.sqrMagnitude > maxSpeed * maxSpeed)
            {
                b.velocity = b.velocity.normalized * maxSpeed;
            }
            // x_{t+1} = x_t + v_{t+1} * dt
            b.position += b.velocity * dt;

            // keep inside vertical world bounds
            b.position.y = Mathf.Clamp(b.position.y, worldMinY, worldMaxY);

            if (b.velocity.sqrMagnitude > 0.0001f)
            {
                b.forward = b.velocity.normalized;
            }

            boids[i] = b;

            // Update visual object
            if (boidObjects[i] != null)
            {
                boidObjects[i].position = b.position;

                if (b.velocity.sqrMagnitude > 0.0001f)
                {
                    Quaternion targetRot = Quaternion.LookRotation(b.velocity.normalized);
                    boidObjects[i].rotation = Quaternion.Slerp(boidObjects[i].rotation,targetRot,rotationSpeed * dt);
                }
            }
        }
    }

    /// Separation, alignment, cohesion, and wander.
    private void ApplyNeighbourRules(int index)
    {
        BBoid self = boids[index];

        Vector3 forward = self.forward.sqrMagnitude > 0.0001f ? self.forward.normalized :
            (self.velocity.sqrMagnitude > 0.0001f ? self.velocity.normalized : Vector3.forward);

        int neighbourCount = 0;
        Vector3 separationAccum = Vector3.zero;
        Vector3 alignmentAccum = Vector3.zero;
        Vector3 cohesionAccum = Vector3.zero;

        for (int j = 0; j < boids.Length; j++)
        {
            if (j == index) {
                continue;
            }

            BBoid other = boids[j];

            Vector3 toOther = other.position - self.position;
            float sqrDist = toOther.sqrMagnitude;
            if (sqrDist <= 0.0001f || sqrDist > sqrNeighbourDistance){
                continue;
            }

            float dist = Mathf.Sqrt(sqrDist);
            Vector3 dir = toOther / dist;

            // 180° field of view
            float dot = Vector3.Dot(forward, dir);
            if (dot <= 0f){
                continue;
            }
            neighbourCount++;

            // Separation: push away from neighbour
            float sepStrength = Mathf.Clamp01((neighbourDistance - dist) / neighbourDistance);
            separationAccum += (-dir) * sepStrength;

            // Alignment: sum neighbour velocities
            alignmentAccum += other.velocity;

            // Cohesion: sum neighbour positions
            cohesionAccum += other.position;
        }

        Vector3 totalForce = Vector3.zero;

        if (neighbourCount > 0)
        {
            separationAccum /= neighbourCount;
            alignmentAccum /= neighbourCount;
            cohesionAccum /= neighbourCount;

            Vector3 sepDir = separationAccum.sqrMagnitude > 0.0001f ? separationAccum.normalized : Vector3.zero;
            Vector3 alignDir = alignmentAccum.sqrMagnitude > 0.0001f ? alignmentAccum.normalized : Vector3.zero;
            Vector3 cohesionDir = (cohesionAccum - self.position);
            cohesionDir = cohesionDir.sqrMagnitude > 0.0001f ? cohesionDir.normalized : Vector3.zero;

            self.separation = sepDir;
            self.alignment = alignDir;
            self.cohesion = cohesionDir;

            Vector3 v = self.velocity;

            if (sepDir != Vector3.zero)
                totalForce += separationWeight * ((sepDir * boidForceScale) - v);
            if (alignDir != Vector3.zero)
                totalForce += alignmentWeight * ((alignDir * boidForceScale) - v);
            if (cohesionDir != Vector3.zero)
                totalForce += cohesionWeight * ((cohesionDir * boidForceScale) - v);
        }
        else
        {
            // No neighbours – keep moving in current direction
            Vector3 wanderDir = self.velocity.sqrMagnitude > 0.0001f
                ? self.velocity.normalized: forward;

            self.separation = Vector3.zero;
            self.alignment = Vector3.zero;
            self.cohesion = Vector3.zero;

            Vector3 v = self.velocity;
            totalForce += wanderWeight * ((wanderDir * boidForceScale) - v);
        }

        self.currentTotalForce += totalForce;
        boids[index] = self;
    }

    /// Obstacles + world boundary rule.
    private void ApplyObstacleAndWorldRules(int index)
    {
        BBoid self = boids[index];

        Vector3 obstacleAccum = Vector3.zero;

        // Physical obstacles
        Collider[] cols = Physics.OverlapSphere(self.position, obstacleCheckRadius);
        foreach (Collider c in cols)
        {
            if (boidObjects != null && boidObjects[index] != null &&
                c.transform == boidObjects[index])
            {
                continue;
            }
            Vector3 closest = c.ClosestPoint(self.position);
            Vector3 away = self.position - closest;
            if (away.sqrMagnitude > 0.0001f)
            {
                obstacleAccum += away.normalized;
            }
        }

        // World bounds as obstacles
        if (self.position.x > worldMaxX){
            obstacleAccum += Vector3.left;
        }
        else if (self.position.x < worldMinX){
            obstacleAccum += Vector3.right;
        }

        if (self.position.z > worldMaxZ){
            obstacleAccum += Vector3.back;
        }
        else if (self.position.z < worldMinZ){
            obstacleAccum += Vector3.forward;
        }
        if (self.position.y > worldMaxY){
            obstacleAccum += Vector3.down;
        }
        else if (self.position.y < worldMinY){
            obstacleAccum += Vector3.up;
        }
        Vector3 totalForce = Vector3.zero;

        if (obstacleAccum.sqrMagnitude > 0.0001f)
        {
            Vector3 obsDir = obstacleAccum.normalized;
            self.obstacle = obsDir;

            Vector3 v = self.velocity;
            totalForce += obstacleWeight * ((obsDir * boidForceScale) - v);
        }
        else
        {
            self.obstacle = Vector3.zero;
        }

        self.currentTotalForce += totalForce;
        boids[index] = self;
    }

    /// Special goal-following behaviour for boid zero.
    private void ApplyBoidZeroGoalRule()
    {
        if (!boidZeroNavigatingTowardGoal || boidZeroPath == null){
            return;}

        if (boidZeroPath.status != NavMeshPathStatus.PathComplete){
            return;}

        if (boidZeroPath.corners == null || boidZeroPath.corners.Length <= 1){
            return;}

        if (currentCorner >= boidZeroPath.corners.Length){
            boidZeroNavigatingTowardGoal = false;
            boidZeroPath.ClearCorners();
            currentCorner = 0;
            return;}

        BBoid b0 = boids[0];

        Vector3 cornerPos = boidZeroPath.corners[currentCorner];
        Vector3 toCorner = cornerPos - b0.position;
        float dist = toCorner.magnitude;

        if (dist < 1f)
        {
            currentCorner++;
            if (currentCorner >= boidZeroPath.corners.Length)
            {
                boidZeroNavigatingTowardGoal = false;
                boidZeroPath.ClearCorners();
                currentCorner = 0;
                boids[0] = b0;
                return;
            }

            cornerPos = boidZeroPath.corners[currentCorner];
            toCorner = cornerPos - b0.position;
        }

        if (toCorner.sqrMagnitude > 0.0001f)
        {
            Vector3 dir = toCorner.normalized;
            Vector3 desired = dir * goalForceScale; 

            Vector3 steering = goalWeight * (desired - b0.velocity);

            b0.currentTotalForce += steering;
        }

        boids[0] = b0;
    }

    private void Update()
    {
        // Render information for boidzero, useful for debugging forces and path planning
        if (boids == null || boids.Length == 0) 
        {
            return;
        }
        int boidCount = boids.Length;
        for (int i = 1; i < boidCount; i++)
        {
            Vector3 boidNeighbourVec = boids[i].position - boids[0].position;
            if (boidNeighbourVec.sqrMagnitude < sqrNeighbourDistance &&
                Vector3.Dot(boidNeighbourVec, boids[0].forward) > 0f)
            {
                Debug.DrawLine(boids[0].position, boids[i].position, Color.blue);
            }
        }

        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].alignment,  Color.green);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].separation, Color.magenta);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].cohesion,   Color.yellow);
        Debug.DrawLine(boids[0].position, boids[0].position + boids[0].obstacle,   Color.red);

        if (boidZeroPath != null)
        {
            int cornersLength = boidZeroPath.corners.Length;
            for (int i = 0; i < cornersLength - 1; i++)
                Debug.DrawLine(boidZeroPath.corners[i], boidZeroPath.corners[i + 1], Color.black);
        }
    }

    public void SetGoal(Vector3 goal)
    {
        // Path following is not interruptible
        if (boidZeroNavigatingTowardGoal)
        {
            return;
        }
        if (boids == null || boids.Length == 0)
        {
            return;
        }
        boidZeroGoal = goal;

        BBoid b0 = boids[0];

        NavMeshHit startHit;
        if (!NavMesh.SamplePosition(b0.position, out startHit, 20f, NavMesh.AllAreas))
        {
            Debug.Log("StartHit = " + startHit.position);
            return;
        }
        NavMeshHit goalHit;
        if (!NavMesh.SamplePosition(goal, out goalHit, 20f, NavMesh.AllAreas))
        {
            Debug.Log("GoalHit = " + goalHit.position);
            return;
        }
        boidZeroPath = new NavMeshPath();

        if (!NavMesh.CalculatePath(startHit.position, goalHit.position, NavMesh.AllAreas, boidZeroPath))
        {
            Debug.Log("Corners = " + boidZeroPath.corners.Length);
            return;
        }
        if (boidZeroPath.status != NavMeshPathStatus.PathComplete || boidZeroPath.corners.Length < 2)
        {
            return;
        }
        boidZeroNavigatingTowardGoal = true;
        currentCorner = 1;
    }
}