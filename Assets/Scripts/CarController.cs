using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(Rigidbody))]
public class KartController : MonoBehaviour
{
    [Header("Import parametrs")]
    [SerializeField] private bool _import = false;
    [SerializeField] private KartConfig _kartConfig;

    [Header("Wheel attachment points")]
    [SerializeField] private Transform _frontLeftWheel;
    [SerializeField] private Transform _frontRightWheel;
    [SerializeField] private Transform _rearLeftWheel;
    [SerializeField] private Transform _rearRightWheel;

    [Header("Input (New Input System)")]
    [SerializeField] private InputActionAsset _playerInput;

    [Header("Weight distribution")]
    [SerializeField, Range(0, 1)] private float _frontAxisShare = 0.5f;

    [Header("Engine & drivetrain")]
    [SerializeField] private KartEngine _engine;
    [SerializeField] private float _gearRatio = 8f;
    [SerializeField] private float _drivetrainEfficiency = 0.9f;

    private InputAction _moveAction;
    private float _throttleInput;
    private float _steepInput;
    private bool _handbrakePressed;

    private float _frontLeftNormalForce, _frontRightNormalForce, _rearLeftNormalForce, _rearRightNormalForce;
    private Rigidbody _rigidbody;
    private Vector3 g = Physics.gravity;

    [SerializeField] private float engineTorque = 400f;
    [SerializeField] private float wheelRadius = 0.3f;
    [SerializeField] private float maxSpeed = 20;

    [Header("Steering")]
    [SerializeField] private float maxSteeringAngle;

    private Quaternion frontLeftInitialRot;
    private Quaternion frontRightInitialRot;

    [Header("Tyre friction")]
    [SerializeField] private float frictionCoefficient = 1f;
    [SerializeField] private float lateralStiffnes = 80f;
    [SerializeField] private float rollingResistance = 30f;

    [Header("Handbrake")]
    [SerializeField] private float handbrakeRollingMultiplier = 8f;

    private float speedAlongForward = 0f;
    private float Fx = 0f;
    private float Fy = 0f;

    private void Awake()
    {
        _playerInput.Enable();
        _rigidbody = GetComponent<Rigidbody>();

        var map = _playerInput.FindActionMap("Kart");
        _moveAction = map.FindAction("Move");

        if (_import) Initialize();

        frontLeftInitialRot = _frontLeftWheel.localRotation;
        frontRightInitialRot = _frontRightWheel.localRotation;

        ComputeStaticWheelLoad();
    }

    private void Initialize()
    {
        if (_kartConfig != null)
        {
            _rigidbody.mass = _kartConfig.mass;
            frictionCoefficient = _kartConfig.frictionCoefficient;
            rollingResistance = _kartConfig.rollingResistance;
            maxSteeringAngle = _kartConfig.maxSteerAngle;
            _gearRatio = _kartConfig.gearRatio;
            wheelRadius = _kartConfig.wheelRadius;
            lateralStiffnes = _kartConfig.lateralStiffness;
        }
    }

    private void OnDisable()
    {
        _playerInput.Disable();
    }

    private void Update()
    {
        ReadInput();
        RotateFrontWheels();
    }

    private void ReadInput()
    {
        Vector2 move = _moveAction.ReadValue<Vector2>();
        _steepInput = Mathf.Clamp(move.x, -1, 1);
        _throttleInput = Mathf.Clamp(move.y, -1, 1);

        if (Input.GetKey(KeyCode.S))
            _throttleInput = -1f;

        _handbrakePressed = Input.GetKey(KeyCode.Space);
    }

    void RotateFrontWheels()
    {
        float steerAngle = maxSteeringAngle * _steepInput;
        Quaternion steerRot = Quaternion.Euler(0, steerAngle, 0);
        _frontLeftWheel.localRotation = frontLeftInitialRot * steerRot;
        _frontRightWheel.localRotation = frontRightInitialRot * steerRot;
    }

    void ComputeStaticWheelLoad()
    {
        float mass = _rigidbody.mass;
        float totalWeight = mass * Mathf.Abs(g.y);
        float frontWeight = totalWeight * _frontAxisShare;
        float rearWeight = totalWeight - frontWeight;

        _frontRightNormalForce = _frontLeftNormalForce = frontWeight * 0.5f;
        _rearRightNormalForce = _rearLeftNormalForce = rearWeight * 0.5f;
    }

    private void ApplyEngineForces()
    {
        Vector3 forward = transform.forward;
        float speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, forward);
        if (_throttleInput > 0 && speedAlongForward > maxSpeed) return;

        float driveTorque = engineTorque * _throttleInput;
        float driveForcePerWheel = driveTorque / wheelRadius / 2;
        Vector3 forceRear = forward * driveForcePerWheel;

        _rigidbody.AddForceAtPosition(forceRear, _rearLeftWheel.position, ForceMode.Force);
        _rigidbody.AddForceAtPosition(forceRear, _rearRightWheel.position, ForceMode.Force);
    }

    private void FixedUpdate()
    {
        ApplyEngineForces();

        ApplyWheelForce(_frontLeftWheel, _frontLeftNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_frontRightWheel, _frontRightNormalForce, isSteer: true, isDrive: false);
        ApplyWheelForce(_rearLeftWheel, _rearLeftNormalForce, isSteer: false, isDrive: true);
        ApplyWheelForce(_rearRightWheel, _rearRightNormalForce, isSteer: false, isDrive: true);
    }

    void ApplyWheelForce(Transform wheel, float normalForce, bool isSteer, bool isDrive)
    {
        Vector3 wheelPos = wheel.position;
        Vector3 wheelForward = wheel.forward;
        Vector3 wheelRight = wheel.right;
        Vector3 velocity = _rigidbody.GetPointVelocity(wheelPos);
        float vlong = Vector3.Dot(velocity, wheelForward);
        float vlat = Vector3.Dot(velocity, wheelRight);

        Fx = 0f;
        Fy = 0f;

        bool isRear = (wheel == _rearLeftWheel || wheel == _rearRightWheel);

        float currentLateralStiffness = lateralStiffnes;
        float currentRollingResistance = rollingResistance;

        if (isRear && _handbrakePressed)
        {
            currentLateralStiffness = 0f;

            currentRollingResistance *= handbrakeRollingMultiplier;

            float additionalBrakeForce = 1200f;
            if (Mathf.Abs(vlong) > 0.5f)
            {
                float brakeDir = vlong > 0 ? -1f : 1f;
                Fx += brakeDir * additionalBrakeForce;
            }
        }

        if (isDrive)
        {
            speedAlongForward = Vector3.Dot(_rigidbody.linearVelocity, transform.forward);
            float engineTorqueOut = _engine.Simulate(_throttleInput, speedAlongForward, Time.fixedDeltaTime);
            float totalWheelTorque = engineTorqueOut * _gearRatio * _drivetrainEfficiency;
            float wheelTorque = totalWheelTorque * 0.5f;
            Fx += wheelTorque / wheelRadius;
        }

        if (isSteer)
        {
            float rooling = -currentRollingResistance * vlong;
            Fx += rooling;
        }

        float fyRaw = -currentLateralStiffness * vlat;
        Fy += fyRaw;

        float frictionlimit = frictionCoefficient * normalForce;
        float forceLenght = Mathf.Sqrt(Fx * Fx + Fy * Fy);
        if (forceLenght > frictionlimit)
        {
            float scale = frictionlimit / forceLenght;
            Fy += scale;
            Fx += scale;
        }

        Vector3 force = wheelForward * Fx + wheelRight * Fy;
        _rigidbody.AddForceAtPosition(force, wheel.position, ForceMode.Force);
    }

    void OnGUI()
    {
        Color panelBg = new Color(0f, 0f, 0f, 0.65f);
        Color sectionBg = new Color(1f, 1f, 1f, 0.06f);
        Color barBg = new Color(1f, 1f, 1f, 0.08f);
        Color barFillSpeed = new Color(0.2f, 0.7f, 1f, 0.95f);
        Color barFillRpm = new Color(0.3f, 1f, 0.5f, 0.95f);
        Color barFillTorque = new Color(1f, 0.8f, 0.2f, 0.95f);

        GUIStyle titleStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 26,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleLeft
        };
        titleStyle.normal.textColor = Color.cyan;

        GUIStyle sectionTitle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 16,
            fontStyle = FontStyle.Bold
        };
        sectionTitle.normal.textColor = new Color(0.8f, 0.9f, 1f, 0.95f);

        GUIStyle valueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 18,
            fontStyle = FontStyle.Normal
        };
        valueStyle.normal.textColor = Color.white;

        GUIStyle subValueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 16,
            fontStyle = FontStyle.Normal
        };
        subValueStyle.normal.textColor = new Color(0.85f, 0.9f, 1f, 0.9f);

        GUIStyle warnStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 18,
            fontStyle = FontStyle.Bold,
            alignment = TextAnchor.MiddleCenter
        };
        warnStyle.normal.textColor = Color.yellow;

        GUIStyle forceStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 16,
            fontStyle = FontStyle.Normal
        };
        forceStyle.normal.textColor = new Color(1f, 0.8f, 1f, 1f);

        Texture2D panelTex = MakeTex(2, 2, panelBg);
        Texture2D sectionTex = MakeTex(2, 2, sectionBg);
        Texture2D barBgTex = MakeTex(2, 2, barBg);
        Texture2D speedFillTex = MakeTex(2, 2, barFillSpeed);
        Texture2D rpmFillTex = MakeTex(2, 2, barFillRpm);
        Texture2D torqueFillTex = MakeTex(2, 2, barFillTorque);

        var panelRect = new Rect(30, 30, 480, 640);
        GUI.DrawTexture(panelRect, panelTex, ScaleMode.StretchToFill);
        GUILayout.BeginArea(panelRect, new GUIStyle { padding = new RectOffset(18, 18, 18, 18) });

        GUILayout.Label("TELEMETRY", titleStyle);
        GUILayout.Space(8);

        DrawSection("SPEED", sectionTitle, sectionTex, () =>
        {
            float kmh = speedAlongForward * 3.6f;
            GUILayout.Label($"{speedAlongForward:0.0} m/s", valueStyle);
            GUILayout.Label($"({kmh:0.0} km/h)", subValueStyle);

            float speedMax = 80f;
            DrawProgress(barBgTex, speedFillTex, Mathf.Clamp01(speedAlongForward / speedMax), 10);
        });

        DrawSection("ENGINE RPM", sectionTitle, sectionTex, () =>
        {
            GUILayout.Label($"{_engine.CurrentRpm:0} RPM", valueStyle);

            float rpmMax = 14000f;
            DrawProgress(barBgTex, rpmFillTex, Mathf.Clamp01(_engine.CurrentRpm / rpmMax), 10);
        });

        DrawSection("ENGINE TORQUE", sectionTitle, sectionTex, () =>
        {
            bool highTorque = _engine.CurrentTorque > 300f;
            Color torqueLabel = highTorque ? new Color(0.6f, 1f, 0.6f, 1f) : new Color(1f, 0.95f, 0.6f, 1f);
            var torqueValStyle = new GUIStyle(valueStyle);
            torqueValStyle.normal.textColor = torqueLabel;

            GUILayout.Label($"{_engine.CurrentTorque:0.0} N·m", torqueValStyle);

            float torqueMax = 500f;
            var fillForTorque = highTorque ? speedFillTex : torqueFillTex;
            DrawProgress(barBgTex, fillForTorque, Mathf.Clamp01(_engine.CurrentTorque / torqueMax), 10);
        });

        if (_handbrakePressed)
        {
            DrawSection("HANDBRAKE", sectionTitle, sectionTex, () =>
            {
                Color prev = GUI.color;
                GUI.color = Color.red;
                GUILayout.Label("HANDBRAKE ACTIVE", warnStyle);
                GUILayout.Label("DRIFT MODE", warnStyle);
                GUI.color = prev;
            });
        }

        DrawSection("WHEEL FORCES", sectionTitle, sectionTex, () =>
        {
            DrawStatRow("Longitudinal", $"{Fx:0.0} N", forceStyle, valueStyle);
            DrawStatRow("Lateral", $"{Fy:0.0} N", forceStyle, valueStyle);
        });

        GUILayout.EndArea();

        void DrawSection(string caption, GUIStyle captionStyle, Texture2D bgTex, System.Action body)
        {
            var bgStyle = new GUIStyle(GUI.skin.box)
            {
                normal = { background = bgTex },
                padding = new RectOffset(12, 12, 10, 12),
                margin = new RectOffset(0, 0, 8, 8)
            };
            GUILayout.BeginVertical(bgStyle);
            GUILayout.Label(caption, captionStyle);
            GUILayout.Space(4);
            body?.Invoke();
            GUILayout.EndVertical();
        }

        void DrawProgress(Texture2D bgTex2, Texture2D fillTex, float t, int height)
        {
            Rect r = GUILayoutUtility.GetRect(GUIContent.none, GUIStyle.none, GUILayout.Height(height), GUILayout.ExpandWidth(true));
            GUI.DrawTexture(r, bgTex2, ScaleMode.StretchToFill);
            float w = Mathf.Round(r.width * Mathf.Clamp01(t));
            if (w > 0.5f)
            {
                var fillRect = new Rect(r.x, r.y, w, r.height);
                GUI.DrawTexture(fillRect, fillTex, ScaleMode.StretchToFill);
            }
        }

        void DrawStatRow(string label, string value, GUIStyle labelStyle, GUIStyle valStyle)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(label, labelStyle, GUILayout.Width(140));
            GUILayout.FlexibleSpace();
            GUILayout.Label(value, valStyle);
            GUILayout.EndHorizontal();
        }
    }

    private Texture2D MakeTex(int width, int height, Color col)
    {
        Color[] pix = new Color[width * height];
        for (int i = 0; i < pix.Length; ++i) pix[i] = col;
        Texture2D result = new Texture2D(width, height);
        result.SetPixels(pix);
        result.Apply();
        return result;
    }
}
