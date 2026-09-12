// Étude Engineers - FTC BioBuzz Blockly Robot Simulator Engine
// Developed for 2026-2027 FTC BIOBUZZ Season (Kickoff Launch Update)

// -------------------------------------------------------------
// 1. Simulation Constants & BioBuzz Field Setup
// -------------------------------------------------------------
const FIELD_SIZE_INCHES = 144; // 12ft x 12ft = 144" x 144" (-72 to +72)
const ROBOT_WIDTH_INCHES = 16;
const ROBOT_LENGTH_INCHES = 16;
const CAMERA_FOV_DEG = 65;
const CAMERA_MAX_RANGE_INCHES = 60;

// Official BioBuzz AprilTags (Perimeter + Central Hive Clusters)
const APRIL_TAGS = [
  // Hive Red Cluster (Audience / Stage)
  { id: 0, x: -16, y: 10, heading: 0, label: "Hive Red Tag 0" },
  { id: 1, x: -16, y: -10, heading: 0, label: "Hive Red Tag 1" },
  { id: 4, x: -8, y: 0, heading: 0, label: "Hive Red Stage Tag 4" },
  
  // Hive Blue Cluster
  { id: 38, x: 16, y: 10, heading: 180, label: "Hive Blue Tag 38" },
  { id: 39, x: 16, y: -10, heading: 180, label: "Hive Blue Tag 39" },
  { id: 40, x: 8, y: 0, heading: 180, label: "Hive Blue Stage Tag 40" },

  // Perimeter Flowers / Wall Tags
  { id: 10, x: -70, y: 0, heading: 0, label: "West Flower Tag 10" },
  { id: 11, x: 70, y: 0, heading: 180, label: "East Flower Tag 11" },
  { id: 12, x: 0, y: 70, heading: -90, label: "North Wall Tag 12" },
  { id: 13, x: 0, y: -70, heading: 90, label: "South Wall Tag 13" }
];

// Staged BioBuzz Game Elements
const FIELD_POLLEN_ELEMENTS = [
  { x: -24, y: 0, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: -30, y: 0, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: -36, y: 0, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: 24, y: 0, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: 30, y: 0, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: 36, y: 0, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: -48, y: 24, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: 48, y: 24, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: 0, y: 48, color: '#facc15', size: 2.8, label: 'Pollen' },
  { x: 0, y: -48, color: '#facc15', size: 2.8, label: 'Pollen' }
];

const FIELD_NECTAR_ELEMENTS = [
  { x: -62, y: -62, color: '#ef4444', size: 3.6, label: 'Red Nectar' },
  { x: -56, y: -62, color: '#ef4444', size: 3.6, label: 'Red Nectar' },
  { x: 62, y: -62, color: '#3b82f6', size: 3.6, label: 'Blue Nectar' },
  { x: 56, y: -62, color: '#3b82f6', size: 3.6, label: 'Blue Nectar' }
];

// Constants for Motor Encoders & Mecanum Kinematics
// goBILDA 5202 Yellow Jacket (19.2:1) = 537.7 ticks/revolution
// 96mm (3.7795") Mecanum Wheel Circumference = pi * 3.7795 = 11.8737 inches
// Ticks Per Inch = 537.7 / 11.8737 = ~45.285 ticks/inch
const TICKS_PER_INCH = 45.285;
const WHEEL_DIAMETER_INCHES = 3.7795;
const MOTOR_TICKS_PER_REV = 537.7;

// Robot State
let robot = {
  // True Physical Pose on Field
  x: -48, // inches (-72 to 72)
  y: -58,
  heading: 90, // degrees (0 = East, 90 = North, 180 = West, 270 = South)
  vx: 0,
  vy: 0,
  vTheta: 0,
  
  // Starting pose for Auto scoring checks
  startX: -48,
  startY: -58,
  
  // Motor Powers (-1.0 to 1.0)
  flPower: 0,
  frPower: 0,
  blPower: 0,
  brPower: 0,

  // Built-in Motor Encoders (Ticks on each motor shaft)
  motorTicksFL: 0,
  motorTicksFR: 0,
  motorTicksBL: 0,
  motorTicksBR: 0,
  motorTicks: 0, // Average

  // Motor Encoder-Based Odometry (Position integrated strictly from 4 motor ticks)
  motorOdoX: -48,
  motorOdoY: -58,
  motorOdoHeading: 90,
  
  // Dead-Wheel Odometry Pods (High precision unpowered tracking wheels)
  odoX: -48,
  odoY: -58,
  odoHeading: 90,
  deadWheelDriftFactor: 0.005, // very low slip
  motorWheelSlipFactor: 0.08,  // wheel slip under motor torque

  // Vision
  visibleTags: [],

  // OpMode State Machine
  currentState: "STATE_START",
  stateStartTime: 0,

  // Auto Match Scoring Status
  autoLeaveScored: false,
  autoParkScored: false,
  autoScore: 0
};

let opModeRunning = false;
let opModeExecutionInterval = null;
let telemetryBuffer = {};
let opModeCodeFunction = null;
let simStartTime = Date.now();

// -------------------------------------------------------------
// 2. Custom FTC Blockly Blocks Definitions
// -------------------------------------------------------------
function registerFTCBlocks() {
  
  // Drivetrain Blocks
  Blockly.Blocks['ftc_mecanum_drive'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("🏎️ Mecanum Drive (Cartesian)");
      this.appendValueInput("DRIVE")
          .setCheck("Number")
          .appendField("Forward (-1 to 1)");
      this.appendValueInput("STRAFE")
          .setCheck("Number")
          .appendField("Strafe (-1 to 1)");
      this.appendValueInput("TURN")
          .setCheck("Number")
          .appendField("Turn (-1 to 1)");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#eab308");
      this.setTooltip("Drives the robot using Mecanum vector kinematics (Forward, Strafe Right/Left, Turn).");
    }
  };

  Blockly.Blocks['ftc_set_motor_powers'] = {
    init: function() {
      this.appendDummyInput().appendField("⚙️ Set 4 Motor Powers (FL, FR, BL, BR)");
      this.appendValueInput("FL").setCheck("Number").appendField("Front Left");
      this.appendValueInput("FR").setCheck("Number").appendField("Front Right");
      this.appendValueInput("BL").setCheck("Number").appendField("Back Left");
      this.appendValueInput("BR").setCheck("Number").appendField("Back Right");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#eab308");
      this.setTooltip("Sets individual motor power levels (-1.0 to 1.0). Useful for understanding strafing sign combinations!");
    }
  };

  Blockly.Blocks['ftc_stop_robot'] = {
    init: function() {
      this.appendDummyInput().appendField("🛑 Stop All Motors");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#eab308");
    }
  };

  Blockly.Blocks['ftc_turn_to_angle'] = {
    init: function() {
      this.appendValueInput("TARGET_ANGLE")
          .setCheck("Number")
          .appendField("🔄 Turn to Heading Angle (°)");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#eab308");
    }
  };

  // Motor Encoders & Odometry Math Blocks
  Blockly.Blocks['ftc_get_motor_ticks_select'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("⚡ Motor Encoder Ticks (")
          .appendField(new Blockly.FieldDropdown([
            ["Average (All 4)", "AVG"],
            ["Front Left (FL)", "FL"],
            ["Front Right (FR)", "FR"],
            ["Back Left (BL)", "BL"],
            ["Back Right (BR)", "BR"]
          ]), "MOTOR")
          .appendField(")");
      this.setOutput(true, "Number");
      this.setColour("#f59e0b");
      this.setTooltip("Reads the encoder tick count of the built-in motor sensors (537.7 ticks/rev).");
    }
  };

  Blockly.Blocks['ftc_get_wheel_ticks'] = {
    init: function() {
      this.appendDummyInput().appendField("⚡ Average Motor Encoder Ticks");
      this.setOutput(true, "Number");
      this.setColour("#f59e0b");
    }
  };

  Blockly.Blocks['ftc_reset_motor_encoders'] = {
    init: function() {
      this.appendDummyInput().appendField("🔄 Reset Motor Encoders (Zero Ticks)");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#f59e0b");
      this.setTooltip("Zeroes out built-in motor encoder counts. Always do this at the start of a distance step!");
    }
  };

  Blockly.Blocks['ftc_ticks_to_inches'] = {
    init: function() {
      this.appendValueInput("TICKS")
          .setCheck("Number")
          .appendField("🔢 Convert Ticks to Inches");
      this.setOutput(true, "Number");
      this.setColour("#f59e0b");
      this.setTooltip("Converts raw encoder ticks to real-world inches (ticks / 45.285).");
    }
  };

  Blockly.Blocks['ftc_inches_to_ticks'] = {
    init: function() {
      this.appendValueInput("INCHES")
          .setCheck("Number")
          .appendField("🔢 Convert Inches to Ticks");
      this.setOutput(true, "Number");
      this.setColour("#f59e0b");
      this.setTooltip("Calculates target encoder ticks for a given distance in inches (inches * 45.285).");
    }
  };

  Blockly.Blocks['ftc_get_motor_odo_x'] = {
    init: function() {
      this.appendDummyInput().appendField("📍 Motor Encoder Odometry X (inches)");
      this.setOutput(true, "Number");
      this.setColour("#f59e0b");
      this.setTooltip("Field X coordinate calculated via forward kinematics from 4 motor encoders.");
    }
  };

  Blockly.Blocks['ftc_get_motor_odo_y'] = {
    init: function() {
      this.appendDummyInput().appendField("📍 Motor Encoder Odometry Y (inches)");
      this.setOutput(true, "Number");
      this.setColour("#f59e0b");
      this.setTooltip("Field Y coordinate calculated via forward kinematics from 4 motor encoders.");
    }
  };

  Blockly.Blocks['ftc_get_motor_odo_heading'] = {
    init: function() {
      this.appendDummyInput().appendField("🧭 Motor Encoder Heading (°)");
      this.setOutput(true, "Number");
      this.setColour("#f59e0b");
    }
  };

  // Dead-Wheel Odometry Blocks
  Blockly.Blocks['ftc_get_odo_x'] = {
    init: function() {
      this.appendDummyInput().appendField("📍 Dead-Wheel Odo X (inches)");
      this.setOutput(true, "Number");
      this.setColour("#3b82f6");
    }
  };

  Blockly.Blocks['ftc_get_odo_y'] = {
    init: function() {
      this.appendDummyInput().appendField("📍 Dead-Wheel Odo Y (inches)");
      this.setOutput(true, "Number");
      this.setColour("#3b82f6");
    }
  };

  Blockly.Blocks['ftc_get_odo_heading'] = {
    init: function() {
      this.appendDummyInput().appendField("🧭 Dead-Wheel / IMU Heading (°)");
      this.setOutput(true, "Number");
      this.setColour("#3b82f6");
    }
  };

  Blockly.Blocks['ftc_reset_odometry'] = {
    init: function() {
      this.appendDummyInput().appendField("🔄 Reset Dead-Wheel Odometry Pose");
      this.appendValueInput("X").setCheck("Number").appendField("X");
      this.appendValueInput("Y").setCheck("Number").appendField("Y");
      this.appendValueInput("HEADING").setCheck("Number").appendField("Heading");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#3b82f6");
    }
  };

  // Vision / AprilTags Blocks
  Blockly.Blocks['ftc_is_apriltag_visible'] = {
    init: function() {
      this.appendValueInput("TAG_ID")
          .setCheck("Number")
          .appendField("👁️ Is AprilTag #");
      this.appendDummyInput().appendField("Visible?");
      this.setOutput(true, "Boolean");
      this.setColour("#10b981");
    }
  };

  Blockly.Blocks['ftc_get_apriltag_distance'] = {
    init: function() {
      this.appendValueInput("TAG_ID")
          .setCheck("Number")
          .appendField("📏 AprilTag #");
      this.appendDummyInput().appendField("Distance (inches)");
      this.setOutput(true, "Number");
      this.setColour("#10b981");
    }
  };

  Blockly.Blocks['ftc_get_apriltag_bearing'] = {
    init: function() {
      this.appendValueInput("TAG_ID")
          .setCheck("Number")
          .appendField("📐 AprilTag #");
      this.appendDummyInput().appendField("Bearing Angle (°)");
      this.setOutput(true, "Number");
      this.setColour("#10b981");
    }
  };

  Blockly.Blocks['ftc_get_apriltag_id'] = {
    init: function() {
      this.appendDummyInput().appendField("🎯 Closest Visible AprilTag ID");
      this.setOutput(true, "Number");
      this.setColour("#10b981");
    }
  };

  Blockly.Blocks['ftc_relocalize_with_apriltag'] = {
    init: function() {
      this.appendValueInput("TAG_ID")
          .setCheck("Number")
          .appendField("🎯 VisionPortal: Relocalize Odometry with Tag #");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#10b981");
      this.setTooltip("Uses camera distance & bearing to the known AprilTag position to zero out accumulated odometry drift.");
    }
  };

  // State Machine Blocks
  Blockly.Blocks['ftc_set_state'] = {
    init: function() {
      this.appendValueInput("STATE")
          .setCheck("String")
          .appendField("⚙️ Switch State to");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#8b5cf6");
    }
  };

  Blockly.Blocks['ftc_get_state'] = {
    init: function() {
      this.appendDummyInput().appendField("⚙️ Current State");
      this.setOutput(true, "String");
      this.setColour("#8b5cf6");
    }
  };

  Blockly.Blocks['ftc_is_state'] = {
    init: function() {
      this.appendValueInput("STATE")
          .setCheck("String")
          .appendField("⚙️ In State?");
      this.setOutput(true, "Boolean");
      this.setColour("#8b5cf6");
    }
  };

  Blockly.Blocks['ftc_get_timer'] = {
    init: function() {
      this.appendDummyInput().appendField("⏱️ State Elapsed Time (sec)");
      this.setOutput(true, "Number");
      this.setColour("#8b5cf6");
    }
  };

  Blockly.Blocks['ftc_reset_timer'] = {
    init: function() {
      this.appendDummyInput().appendField("⏱️ Reset State Timer");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#8b5cf6");
    }
  };

  // Telemetry & Scoring Blocks
  Blockly.Blocks['ftc_telemetry_add_data'] = {
    init: function() {
      this.appendValueInput("CAPTION").setCheck("String").appendField("📊 telemetry.addData(");
      this.appendValueInput("VALUE").appendField(",");
      this.appendDummyInput().appendField(")");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#ec4899");
    }
  };

  Blockly.Blocks['ftc_telemetry_update'] = {
    init: function() {
      this.appendDummyInput().appendField("📊 telemetry.update()");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#ec4899");
    }
  };

  Blockly.Blocks['ftc_get_auto_score'] = {
    init: function() {
      this.appendDummyInput().appendField("🏆 Get Auto Match Points (0-8)");
      this.setOutput(true, "Number");
      this.setColour("#ec4899");
      this.setTooltip("Returns current Auto points earned (Leave = 3 pts, Auto Park = 5 pts).");
    }
  };

  // Loop & Control
  Blockly.Blocks['ftc_opmode_is_active'] = {
    init: function() {
      this.appendDummyInput().appendField("🔄 opModeIsActive()");
      this.setOutput(true, "Boolean");
      this.setColour("#0284c7");
    }
  };

  Blockly.Blocks['ftc_sleep_ms'] = {
    init: function() {
      this.appendValueInput("MS").setCheck("Number").appendField("⏳ sleep (ms)");
      this.setPreviousStatement(true, null);
      this.setNextStatement(true, null);
      this.setColour("#0284c7");
    }
  };

  // -------------------------------------------------------------
  // JavaScript Generators
  // -------------------------------------------------------------
  const jsGen = (typeof javascript !== 'undefined' && javascript.javascriptGenerator)
    ? javascript.javascriptGenerator
    : (typeof Blockly !== 'undefined' && (Blockly.JavaScript || Blockly.javascriptGenerator));
  
  const jsOrder = (typeof javascript !== 'undefined' && javascript.Order)
    ? javascript.Order
    : (typeof Blockly !== 'undefined' && Blockly.JavaScript ? {
        ATOMIC: Blockly.JavaScript.ORDER_ATOMIC || 0,
        FUNCTION_CALL: Blockly.JavaScript.ORDER_FUNCTION_CALL || 2,
        EQUALITY: Blockly.JavaScript.ORDER_EQUALITY || 7
      } : { ATOMIC: 0, FUNCTION_CALL: 2, EQUALITY: 7 });

  if (jsGen && jsGen.forBlock) {
    jsGen.forBlock['ftc_mecanum_drive'] = function(block, generator) {
      var gen = generator || jsGen;
      var drive = gen.valueToCode(block, 'DRIVE', jsOrder.ATOMIC) || '0';
      var strafe = gen.valueToCode(block, 'STRAFE', jsOrder.ATOMIC) || '0';
      var turn = gen.valueToCode(block, 'TURN', jsOrder.ATOMIC) || '0';
      return `simMecanumDrive(${drive}, ${strafe}, ${turn});\n`;
    };

    jsGen.forBlock['ftc_set_motor_powers'] = function(block, generator) {
      var gen = generator || jsGen;
      var fl = gen.valueToCode(block, 'FL', jsOrder.ATOMIC) || '0';
      var fr = gen.valueToCode(block, 'FR', jsOrder.ATOMIC) || '0';
      var bl = gen.valueToCode(block, 'BL', jsOrder.ATOMIC) || '0';
      var br = gen.valueToCode(block, 'BR', jsOrder.ATOMIC) || '0';
      return `simSetMotorPowers(${fl}, ${fr}, ${bl}, ${br});\n`;
    };

    jsGen.forBlock['ftc_stop_robot'] = function() {
      return `simStopRobot();\n`;
    };

    jsGen.forBlock['ftc_turn_to_angle'] = function(block, generator) {
      var gen = generator || jsGen;
      var angle = gen.valueToCode(block, 'TARGET_ANGLE', jsOrder.ATOMIC) || '0';
      return `simTurnToAngle(${angle});\n`;
    };

    jsGen.forBlock['ftc_get_motor_ticks_select'] = function(block) {
      var motor = block.getFieldValue('MOTOR') || 'AVG';
      return [`simGetMotorTicks('${motor}')`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_wheel_ticks'] = function() {
      return [`simGetMotorTicks('AVG')`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_reset_motor_encoders'] = function() {
      return `simResetMotorEncoders();\n`;
    };

    jsGen.forBlock['ftc_ticks_to_inches'] = function(block, generator) {
      var gen = generator || jsGen;
      var ticks = gen.valueToCode(block, 'TICKS', jsOrder.ATOMIC) || '0';
      return [`simTicksToInches(${ticks})`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_inches_to_ticks'] = function(block, generator) {
      var gen = generator || jsGen;
      var inches = gen.valueToCode(block, 'INCHES', jsOrder.ATOMIC) || '0';
      return [`simInchesToTicks(${inches})`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_motor_odo_x'] = function() {
      return [`simGetMotorOdoX()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_motor_odo_y'] = function() {
      return [`simGetMotorOdoY()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_motor_odo_heading'] = function() {
      return [`simGetMotorOdoHeading()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_odo_x'] = function() {
      return [`simGetOdoX()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_odo_y'] = function() {
      return [`simGetOdoY()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_odo_heading'] = function() {
      return [`simGetOdoHeading()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_reset_odometry'] = function(block, generator) {
      var gen = generator || jsGen;
      var x = gen.valueToCode(block, 'X', jsOrder.ATOMIC) || '0';
      var y = gen.valueToCode(block, 'Y', jsOrder.ATOMIC) || '0';
      var h = gen.valueToCode(block, 'HEADING', jsOrder.ATOMIC) || '0';
      return `simResetOdometry(${x}, ${y}, ${h});\n`;
    };

    jsGen.forBlock['ftc_is_apriltag_visible'] = function(block, generator) {
      var gen = generator || jsGen;
      var tagId = gen.valueToCode(block, 'TAG_ID', jsOrder.ATOMIC) || '0';
      return [`simIsAprilTagVisible(${tagId})`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_apriltag_distance'] = function(block, generator) {
      var gen = generator || jsGen;
      var tagId = gen.valueToCode(block, 'TAG_ID', jsOrder.ATOMIC) || '0';
      return [`simGetAprilTagDistance(${tagId})`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_apriltag_bearing'] = function(block, generator) {
      var gen = generator || jsGen;
      var tagId = gen.valueToCode(block, 'TAG_ID', jsOrder.ATOMIC) || '0';
      return [`simGetAprilTagBearing(${tagId})`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_get_apriltag_id'] = function() {
      return [`simGetClosestAprilTagId()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_relocalize_with_apriltag'] = function(block, generator) {
      var gen = generator || jsGen;
      var tagId = gen.valueToCode(block, 'TAG_ID', jsOrder.ATOMIC) || '0';
      return `simRelocalizeWithTag(${tagId});\n`;
    };

    jsGen.forBlock['ftc_set_state'] = function(block, generator) {
      var gen = generator || jsGen;
      var state = gen.valueToCode(block, 'STATE', jsOrder.ATOMIC) || '""';
      return `simSetState(${state});\n`;
    };

    jsGen.forBlock['ftc_get_state'] = function() {
      return [`simGetState()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_is_state'] = function(block, generator) {
      var gen = generator || jsGen;
      var state = gen.valueToCode(block, 'STATE', jsOrder.ATOMIC) || '""';
      return [`(simGetState() === ${state})`, jsOrder.EQUALITY];
    };

    jsGen.forBlock['ftc_get_timer'] = function() {
      return [`simGetStateTimer()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_reset_timer'] = function() {
      return `simResetStateTimer();\n`;
    };

    jsGen.forBlock['ftc_telemetry_add_data'] = function(block, generator) {
      var gen = generator || jsGen;
      var cap = gen.valueToCode(block, 'CAPTION', jsOrder.ATOMIC) || '""';
      var val = gen.valueToCode(block, 'VALUE', jsOrder.ATOMIC) || '""';
      return `simTelemetryAddData(${cap}, ${val});\n`;
    };

    jsGen.forBlock['ftc_telemetry_update'] = function() {
      return `simTelemetryUpdate();\n`;
    };

    jsGen.forBlock['ftc_get_auto_score'] = function() {
      return [`simGetAutoScore()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_opmode_is_active'] = function() {
      return [`simOpModeIsActive()`, jsOrder.FUNCTION_CALL];
    };

    jsGen.forBlock['ftc_sleep_ms'] = function(block, generator) {
      var gen = generator || jsGen;
      var ms = gen.valueToCode(block, 'MS', jsOrder.ATOMIC) || '0';
      return `/* non-blocking sleep simulated */ simSleep(${ms});\n`;
    };
  }
}

// -------------------------------------------------------------
// 3. Robot Simulator API Bridge Functions
// -------------------------------------------------------------
function simMecanumDrive(drive, strafe, turn) {
  drive = Math.max(-1, Math.min(1, drive));
  strafe = Math.max(-1, Math.min(1, strafe));
  turn = Math.max(-1, Math.min(1, turn));

  // Mecanum Kinematics:
  // FL = Drive + Strafe + Turn
  // FR = Drive - Strafe - Turn
  // BL = Drive - Strafe + Turn
  // BR = Drive + Strafe - Turn
  let fl = drive + strafe + turn;
  let fr = drive - strafe - turn;
  let bl = drive - strafe + turn;
  let br = drive + strafe - turn;

  // Normalize
  let max = Math.max(Math.abs(fl), Math.abs(fr), Math.abs(bl), Math.abs(br));
  if (max > 1.0) {
    fl /= max; fr /= max; bl /= max; br /= max;
  }
  simSetMotorPowers(fl, fr, bl, br);
}

function simSetMotorPowers(fl, fr, bl, br) {
  robot.flPower = Math.max(-1, Math.min(1, fl));
  robot.frPower = Math.max(-1, Math.min(1, fr));
  robot.blPower = Math.max(-1, Math.min(1, bl));
  robot.brPower = Math.max(-1, Math.min(1, br));
}

function simTurnToAngle(targetAngle) {
  let diff = targetAngle - robot.heading;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  
  if (Math.abs(diff) < 2) {
    simStopRobot();
    return;
  }
  let turnPower = Math.sign(diff) * Math.min(0.6, Math.max(0.2, Math.abs(diff) / 45));
  simMecanumDrive(0, 0, turnPower);
}

function simStopRobot() {
  robot.flPower = 0;
  robot.frPower = 0;
  robot.blPower = 0;
  robot.brPower = 0;
}

// Motor Encoder Functions
function simGetMotorTicks(motor) {
  if (motor === 'FL') return Math.round(robot.motorTicksFL);
  if (motor === 'FR') return Math.round(robot.motorTicksFR);
  if (motor === 'BL') return Math.round(robot.motorTicksBL);
  if (motor === 'BR') return Math.round(robot.motorTicksBR);
  return Math.round(robot.motorTicks);
}

function simResetMotorEncoders() {
  robot.motorTicksFL = 0;
  robot.motorTicksFR = 0;
  robot.motorTicksBL = 0;
  robot.motorTicksBR = 0;
  robot.motorTicks = 0;
}

function simTicksToInches(ticks) {
  return Math.round((ticks / TICKS_PER_INCH) * 10) / 10;
}

function simInchesToTicks(inches) {
  return Math.round(inches * TICKS_PER_INCH);
}

function simGetMotorOdoX() { return Math.round(robot.motorOdoX * 10) / 10; }
function simGetMotorOdoY() { return Math.round(robot.motorOdoY * 10) / 10; }
function simGetMotorOdoHeading() { return Math.round(robot.motorOdoHeading * 10) / 10; }

// Dead-Wheel Odometry Functions
function simGetOdoX() { return Math.round(robot.odoX * 10) / 10; }
function simGetOdoY() { return Math.round(robot.odoY * 10) / 10; }
function simGetOdoHeading() { return Math.round(robot.odoHeading * 10) / 10; }

function simResetOdometry(x, y, heading) {
  robot.odoX = x;
  robot.odoY = y;
  robot.odoHeading = heading;
}

function simGetWheelTicks() {
  return Math.round(robot.motorTicks);
}

function simIsAprilTagVisible(tagId) {
  return robot.visibleTags.some(t => t.id === tagId);
}

function simGetAprilTagDistance(tagId) {
  let tag = robot.visibleTags.find(t => t.id === tagId);
  return tag ? Math.round(tag.dist * 10) / 10 : 999;
}

function simGetAprilTagBearing(tagId) {
  let tag = robot.visibleTags.find(t => t.id === tagId);
  return tag ? Math.round(tag.bearing * 10) / 10 : 0;
}

function simGetClosestAprilTagId() {
  if (robot.visibleTags.length === 0) return -1;
  return robot.visibleTags[0].id;
}

function simRelocalizeWithTag(tagId) {
  let tag = robot.visibleTags.find(t => t.id === tagId);
  if (!tag) return;
  robot.odoX = robot.x + (Math.random() - 0.5) * 0.2;
  robot.odoY = robot.y + (Math.random() - 0.5) * 0.2;
  robot.odoHeading = robot.heading + (Math.random() - 0.5) * 0.3;
  robot.motorOdoX = robot.x;
  robot.motorOdoY = robot.y;
  robot.motorOdoHeading = robot.heading;
  simTelemetryAddData("VISION_STATUS", `✅ Relocalized with Tag #${tagId} (Drift Reset!)`);
}

function simSetState(state) {
  robot.currentState = state;
  robot.stateStartTime = Date.now();
  const badge = document.getElementById('activeStateBadge');
  if (badge) badge.innerText = state;
}

function simGetState() {
  return robot.currentState;
}

function simGetStateTimer() {
  return (Date.now() - robot.stateStartTime) / 1000.0;
}

function simResetStateTimer() {
  robot.stateStartTime = Date.now();
}

function simGetAutoScore() {
  return robot.autoScore;
}

function simTelemetryAddData(caption, value) {
  telemetryBuffer[caption] = value;
}

function simTelemetryUpdate() {
  const container = document.getElementById('telemetryLines');
  if (!container) return;
  let html = '';
  for (let key in telemetryBuffer) {
    html += `<div class="flex items-center space-x-2"><span class="text-amber-400 font-bold">${key}:</span> <span class="text-emerald-300 font-mono">${telemetryBuffer[key]}</span></div>`;
  }
  container.innerHTML = html || '<div class="text-slate-500 italic">// No telemetry data sent yet</div>';
}

function simOpModeIsActive() {
  return opModeRunning;
}

function simSleep(ms) {}

// -------------------------------------------------------------
// 4. Physics Engine, BioBuzz Auto Scoring & Canvas Renderer
// -------------------------------------------------------------
const canvas = document.getElementById('fieldCanvas');
const ctx = canvas.getContext('2d');

function updatePhysics(dt) {
  const MAX_SPEED_INCHES_PER_SEC = 45; // ~3.75 ft/sec goBILDA Strafer
  const MAX_TURN_DEG_PER_SEC = 240;

  // Forward, Strafe, Turn command from 4 Mecanum wheels
  let driveCmd = (robot.flPower + robot.frPower + robot.blPower + robot.brPower) / 4;
  let strafeCmd = (robot.flPower - robot.frPower - robot.blPower + robot.brPower) / 4;
  let turnCmd = (robot.flPower - robot.frPower + robot.blPower - robot.brPower) / 4;

  let headingRad = (robot.heading * Math.PI) / 180;

  // Robot-relative drive/strafe to field-relative velocities
  let forwardVel = driveCmd * MAX_SPEED_INCHES_PER_SEC;
  let strafeVel = strafeCmd * MAX_SPEED_INCHES_PER_SEC;
  let turnVel = turnCmd * MAX_TURN_DEG_PER_SEC;

  let fieldDx = (forwardVel * Math.sin(headingRad) + strafeVel * Math.cos(headingRad)) * dt;
  let fieldDy = (forwardVel * Math.cos(headingRad) - strafeVel * Math.sin(headingRad)) * dt;
  let dTheta = turnVel * dt;

  // Apply Physics to True Robot Pose
  robot.x += fieldDx;
  robot.y += fieldDy;
  robot.heading = (robot.heading + dTheta + 360) % 360;

  // Constrain inside 12x12ft Field
  const limit = 72 - (ROBOT_WIDTH_INCHES / 2);
  robot.x = Math.max(-limit, Math.min(limit, robot.x));
  robot.y = Math.max(-limit, Math.min(limit, robot.y));

  // 4 Individual Wheel Speeds & Encoders
  let flSpeed = (robot.flPower) * MAX_SPEED_INCHES_PER_SEC;
  let frSpeed = (robot.frPower) * MAX_SPEED_INCHES_PER_SEC;
  let blSpeed = (robot.blPower) * MAX_SPEED_INCHES_PER_SEC;
  let brSpeed = (robot.brPower) * MAX_SPEED_INCHES_PER_SEC;

  let slipNoise = (robot.flPower !== 0 || robot.frPower !== 0 || robot.blPower !== 0 || robot.brPower !== 0) 
    ? (Math.random() - 0.5) * robot.motorWheelSlipFactor : 0;

  let dTicksFL = flSpeed * TICKS_PER_INCH * (1 + slipNoise) * dt;
  let dTicksFR = frSpeed * TICKS_PER_INCH * (1 + slipNoise) * dt;
  let dTicksBL = blSpeed * TICKS_PER_INCH * (1 + slipNoise) * dt;
  let dTicksBR = brSpeed * TICKS_PER_INCH * (1 + slipNoise) * dt;

  robot.motorTicksFL += dTicksFL;
  robot.motorTicksFR += dTicksFR;
  robot.motorTicksBL += dTicksBL;
  robot.motorTicksBR += dTicksBR;
  
  // Average magnitude for forward/strafe monitoring
  robot.motorTicks = (Math.abs(robot.motorTicksFL) + Math.abs(robot.motorTicksFR) + Math.abs(robot.motorTicksBL) + Math.abs(robot.motorTicksBR)) / 4;

  // Forward Kinematics (Dead reckoning strictly from 4 motor ticks)
  let dForwardEnc = (dTicksFL + dTicksFR + dTicksBL + dTicksBR) / (4 * TICKS_PER_INCH);
  let dStrafeEnc = (dTicksFL - dTicksFR - dTicksBL + dTicksBR) / (4 * TICKS_PER_INCH);
  let dHeadingEnc = (-dTicksFL + dTicksFR - dTicksBL + dTicksBR) / (4 * TICKS_PER_INCH * (ROBOT_WIDTH_INCHES / 2)) * (180 / Math.PI);

  let mHeadingRad = (robot.motorOdoHeading * Math.PI) / 180;
  robot.motorOdoX += (dForwardEnc * Math.sin(mHeadingRad) + dStrafeEnc * Math.cos(mHeadingRad));
  robot.motorOdoY += (dForwardEnc * Math.cos(mHeadingRad) - dStrafeEnc * Math.sin(mHeadingRad));
  robot.motorOdoHeading = (robot.motorOdoHeading + dHeadingEnc + 360) % 360;

  // Dead-Wheel Odometry Tracking (Tracking pods)
  let odoSlipX = (Math.random() - 0.5) * robot.deadWheelDriftFactor * Math.abs(driveCmd) * dt;
  let odoSlipY = (Math.random() - 0.5) * robot.deadWheelDriftFactor * Math.abs(driveCmd) * dt;
  robot.odoX += fieldDx + odoSlipX;
  robot.odoY += fieldDy + odoSlipY;
  robot.odoHeading = (robot.odoHeading + dTheta + 360) % 360;

  // Auto Scoring Check (Official BIOBUZZ Rules Table 10-2)
  // 1. AUTO: LEAVE (3 pts): Robot moves >= 8 inches away from starting perimeter wall
  let distFromStart = Math.hypot(robot.x - robot.startX, robot.y - robot.startY);
  if (distFromStart >= 8 && !robot.autoLeaveScored) {
    robot.autoLeaveScored = true;
  }

  // 2. AUTO: PARK (5 pts): Robot is inside Loading Zone (-72 to -49, -72 to -49) or (+49 to 72, -72 to -49)
  let inRedLoadingZone = (robot.x >= -72 && robot.x <= -48 && robot.y >= -72 && robot.y <= -48);
  let inBlueLoadingZone = (robot.x >= 48 && robot.x <= 72 && robot.y >= -72 && robot.y <= -48);
  if ((inRedLoadingZone || inBlueLoadingZone) && distFromStart > 12) {
    robot.autoParkScored = true;
  }

  robot.autoScore = (robot.autoLeaveScored ? 3 : 0) + (robot.autoParkScored ? 5 : 0);

  // Update Vision
  updateVision();

  // Update UI Elements
  const coordDisplay = document.getElementById('fieldCoordsDisplay');
  if (coordDisplay) {
    coordDisplay.innerText = `X: ${robot.x.toFixed(1)}" | Y: ${robot.y.toFixed(1)}" | θ: ${robot.heading.toFixed(1)}°`;
  }
  
  const odoDisplay = document.getElementById('quickOdoDisplay');
  if (odoDisplay) {
    odoDisplay.innerText = `X: ${robot.motorOdoX.toFixed(1)}" | Y: ${robot.motorOdoY.toFixed(1)}" | Ticks: ${Math.round(robot.motorTicks)}`;
  }

  const scoreDisplay = document.getElementById('autoScoreDisplay');
  if (scoreDisplay) {
    scoreDisplay.innerText = robot.autoScore;
  }

  if (opModeRunning) {
    const timerBadge = document.getElementById('stateTimerBadge');
    if (timerBadge) timerBadge.innerText = `${simGetStateTimer().toFixed(2)}s`;
  }
}

function updateVision() {
  robot.visibleTags = [];
  let headingRad = (robot.heading * Math.PI) / 180;

  APRIL_TAGS.forEach(tag => {
    let dx = tag.x - robot.x;
    let dy = tag.y - robot.y;
    let dist = Math.sqrt(dx*dx + dy*dy);

    if (dist <= CAMERA_MAX_RANGE_INCHES) {
      let angleToTag = Math.atan2(dx, dy);
      let diffAngle = angleToTag - headingRad;
      
      while (diffAngle > Math.PI) diffAngle -= 2 * Math.PI;
      while (diffAngle < -Math.PI) diffAngle += 2 * Math.PI;

      let diffDeg = (diffAngle * 180) / Math.PI;

      if (Math.abs(diffDeg) <= CAMERA_FOV_DEG / 2) {
        robot.visibleTags.push({
          id: tag.id,
          label: tag.label,
          dist: dist,
          bearing: diffDeg
        });
      }
    }
  });

  const visionContainer = document.getElementById('aprilTagDetectionsList');
  if (visionContainer) {
    if (robot.visibleTags.length === 0) {
      visionContainer.innerHTML = '<div class="text-slate-500 italic">// No AprilTags in camera field of view.</div>';
    } else {
      let html = '';
      robot.visibleTags.forEach(t => {
        html += `<div class="bg-slate-900 p-1.5 rounded border border-emerald-500/30 flex justify-between items-center text-[11px]">
          <span class="text-emerald-400 font-bold">Tag #${t.id} (${t.label})</span>
          <span class="text-slate-300">Dist: <strong class="text-amber-300">${t.dist.toFixed(1)}"</strong> | Bearing: <strong class="text-amber-300">${t.bearing.toFixed(1)}°</strong></span>
        </div>`;
      });
      visionContainer.innerHTML = html;
    }
  }
}

function inchToCanvas(inchX, inchY) {
  const scale = canvas.width / FIELD_SIZE_INCHES; // 520 / 144 = 3.611 px/inch
  const cx = canvas.width / 2;
  const cy = canvas.height / 2;
  return {
    x: cx + inchX * scale,
    y: cy - inchY * scale
  };
}

function renderField() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // 1. Foam Tile Grid (6x6 tiles of 24" each)
  const tileSize = canvas.width / 6;
  for (let r = 0; r < 6; r++) {
    for (let c = 0; c < 6; c++) {
      ctx.fillStyle = (r + c) % 2 === 0 ? '#1e293b' : '#0f172a';
      ctx.fillRect(c * tileSize, r * tileSize, tileSize, tileSize);
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1;
      ctx.strokeRect(c * tileSize, r * tileSize, tileSize, tileSize);
    }
  }

  // 2. Field Perimeter & Center Grid Axes
  ctx.strokeStyle = 'rgba(100, 116, 139, 0.4)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(canvas.width / 2, 0); ctx.lineTo(canvas.width / 2, canvas.height);
  ctx.moveTo(0, canvas.height / 2); ctx.lineTo(canvas.width, canvas.height / 2);
  ctx.stroke();

  // 3. Official BIOBUZZ™ Alliance Loading & Garden Zones
  // Red Loading Zone (Bottom Left: 23" x 11")
  let rlzTL = inchToCanvas(-72, -49);
  let rlzBR = inchToCanvas(-49, -72);
  ctx.fillStyle = 'rgba(239, 68, 68, 0.25)';
  ctx.fillRect(rlzTL.x, rlzTL.y, rlzBR.x - rlzTL.x, rlzBR.y - rlzTL.y);
  ctx.strokeStyle = '#ef4444';
  ctx.lineWidth = 2;
  ctx.strokeRect(rlzTL.x, rlzTL.y, rlzBR.x - rlzTL.x, rlzBR.y - rlzTL.y);
  ctx.fillStyle = '#f87171';
  ctx.font = 'bold 8px sans-serif';
  ctx.fillText("RED LOADING ZONE (AUTO PARK 5 PTS)", rlzTL.x + 4, rlzBR.y - 6);

  // Blue Loading Zone (Bottom Right: 23" x 11")
  let blzTL = inchToCanvas(49, -49);
  let blzBR = inchToCanvas(72, -72);
  ctx.fillStyle = 'rgba(59, 130, 246, 0.25)';
  ctx.fillRect(blzTL.x, blzTL.y, blzBR.x - blzTL.x, blzBR.y - blzTL.y);
  ctx.strokeStyle = '#3b82f6';
  ctx.lineWidth = 2;
  ctx.strokeRect(blzTL.x, blzTL.y, blzBR.x - blzTL.x, blzBR.y - blzTL.y);
  ctx.fillStyle = '#60a5fa';
  ctx.font = 'bold 8px sans-serif';
  ctx.fillText("BLUE LOADING ZONE", blzTL.x + 4, blzBR.y - 6);

  // 4. Central HIVE Structure (Section 9.6: 49.5" W x 39.0" D)
  let hiveTL = inchToCanvas(-24.73, 19.5);
  let hiveBR = inchToCanvas(24.73, -19.5);
  let hiveW = hiveBR.x - hiveTL.x;
  let hiveH = hiveBR.y - hiveTL.y;

  ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
  ctx.fillRect(hiveTL.x, hiveTL.y, hiveW, hiveH);
  ctx.strokeStyle = '#fbbf24';
  ctx.lineWidth = 2;
  ctx.strokeRect(hiveTL.x, hiveTL.y, hiveW, hiveH);

  // Red Cell & Blue Cell in Hive
  let redCellTL = inchToCanvas(-22, 16);
  let redCellBR = inchToCanvas(-2, -16);
  ctx.fillStyle = 'rgba(239, 68, 68, 0.25)';
  ctx.fillRect(redCellTL.x, redCellTL.y, redCellBR.x - redCellTL.x, redCellBR.y - redCellTL.y);
  ctx.strokeStyle = '#ef4444';
  ctx.strokeRect(redCellTL.x, redCellTL.y, redCellBR.x - redCellTL.x, redCellBR.y - redCellTL.y);

  let blueCellTL = inchToCanvas(2, 16);
  let blueCellBR = inchToCanvas(22, -16);
  ctx.fillStyle = 'rgba(59, 130, 246, 0.25)';
  ctx.fillRect(blueCellTL.x, blueCellTL.y, blueCellBR.x - blueCellTL.x, blueCellBR.y - blueCellTL.y);
  ctx.strokeStyle = '#3b82f6';
  ctx.strokeRect(blueCellTL.x, blueCellTL.y, blueCellBR.x - blueCellTL.x, blueCellBR.y - blueCellTL.y);

  ctx.fillStyle = '#fef08a';
  ctx.font = 'bold 9px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText("🐝 CENTRAL HIVE (BIOBUZZ™)", canvas.width / 2, canvas.height / 2 - 4);
  ctx.font = '8px sans-serif';
  ctx.fillStyle = '#94a3b8';
  ctx.fillText("Bi-Stable Flip Tip Targets", canvas.width / 2, canvas.height / 2 + 8);
  ctx.textAlign = 'left';

  // 5. Four Perimeter FLOWERS (Section 9.7)
  const flowers = [
    { x: 0, y: 68, label: "North Flower" },
    { x: 0, y: -68, label: "South Flower" },
    { x: -68, y: 0, label: "West Flower (Station 1)" },
    { x: 68, y: 0, label: "East Flower (Station 2)" }
  ];

  flowers.forEach(fl => {
    let fp = inchToCanvas(fl.x, fl.y);
    ctx.save();
    ctx.translate(fp.x, fp.y);

    // Petals
    ctx.fillStyle = 'rgba(236, 72, 153, 0.3)';
    for (let a = 0; a < 6; a++) {
      let rad = (a * 60 * Math.PI) / 180;
      ctx.beginPath();
      ctx.arc(Math.cos(rad) * 10, Math.sin(rad) * 10, 6, 0, Math.PI * 2);
      ctx.fill();
    }
    // Flower Center
    ctx.fillStyle = '#ec4899';
    ctx.beginPath();
    ctx.arc(0, 0, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.stroke();

    ctx.fillStyle = '#fdf2f8';
    ctx.font = 'bold 7px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText("FLOWER", 0, 15);
    ctx.restore();
  });

  // 6. Staged Pollen & Nectar Elements
  FIELD_POLLEN_ELEMENTS.forEach(p => {
    let pp = inchToCanvas(p.x, p.y);
    let scale = canvas.width / FIELD_SIZE_INCHES;
    let r = (p.size / 2) * scale;
    ctx.fillStyle = p.color;
    ctx.beginPath();
    ctx.arc(pp.x, pp.y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ca8a04';
    ctx.lineWidth = 1;
    ctx.stroke();
  });

  FIELD_NECTAR_ELEMENTS.forEach(n => {
    let np = inchToCanvas(n.x, n.y);
    let scale = canvas.width / FIELD_SIZE_INCHES;
    let r = (n.size / 2) * scale;
    ctx.fillStyle = n.color;
    ctx.beginPath();
    ctx.arc(np.x, np.y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  });

  // 7. Mission-Specific Waypoints & Corridors
  if (currentMissionKey === 'mission_strafing') {
    // Waypoint 1: Strafe Target (X: -20, Y: -48)
    let wp1 = inchToCanvas(-20, -48);
    let distWp1 = Math.hypot(robot.x - (-20), robot.y - (-48));
    ctx.save();
    ctx.beginPath();
    ctx.arc(wp1.x, wp1.y, 16, 0, Math.PI * 2);
    ctx.fillStyle = distWp1 < 5 ? 'rgba(16, 185, 129, 0.5)' : 'rgba(245, 158, 11, 0.25)';
    ctx.fill();
    ctx.strokeStyle = distWp1 < 5 ? '#10b981' : '#f59e0b';
    ctx.lineWidth = 2;
    ctx.setLineDash([3, 3]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#f59e0b';
    ctx.font = 'bold 9px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText("WP1: Strafe East (X: -20, Y: -48)", wp1.x, wp1.y - 18);
    ctx.restore();

    // Waypoint 2: Strafe West to Loading Zone (X: -60, Y: -48)
    let wp2 = inchToCanvas(-60, -48);
    let distWp2 = Math.hypot(robot.x - (-60), robot.y - (-48));
    ctx.save();
    ctx.beginPath();
    ctx.arc(wp2.x, wp2.y, 16, 0, Math.PI * 2);
    ctx.fillStyle = distWp2 < 5 ? 'rgba(16, 185, 129, 0.5)' : 'rgba(239, 68, 68, 0.25)';
    ctx.fill();
    ctx.strokeStyle = distWp2 < 5 ? '#10b981' : '#ef4444';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#ef4444';
    ctx.font = 'bold 9px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText("WP2: Strafe West to Base (X: -60, Y: -48)", wp2.x, wp2.y + 24);
    ctx.restore();
  } else if (currentMissionKey === 'mission_encoder_odometry') {
    // Waypoint 1: (X: -48, Y: -12) -> 36" Forward
    let wp1 = inchToCanvas(-48, -12);
    let distWp1 = Math.hypot(robot.x - (-48), robot.y - (-12));
    ctx.save();
    ctx.beginPath();
    ctx.arc(wp1.x, wp1.y, 14, 0, Math.PI * 2);
    ctx.fillStyle = distWp1 < 5 ? 'rgba(16, 185, 129, 0.5)' : 'rgba(245, 158, 11, 0.25)';
    ctx.fill();
    ctx.strokeStyle = distWp1 < 5 ? '#10b981' : '#f59e0b';
    ctx.lineWidth = 2;
    ctx.setLineDash([3, 3]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#f59e0b';
    ctx.font = 'bold 9px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText("WP1: 36\" Fwd (1630 Ticks)", wp1.x, wp1.y - 18);
    ctx.restore();

    // Waypoint 2: (X: 0, Y: -12) -> 48" Strafe Right to Hive
    let wp2 = inchToCanvas(0, -12);
    let distWp2 = Math.hypot(robot.x - 0, robot.y - (-12));
    ctx.save();
    ctx.beginPath();
    ctx.arc(wp2.x, wp2.y, 16, 0, Math.PI * 2);
    ctx.fillStyle = distWp2 < 5 ? 'rgba(16, 185, 129, 0.5)' : 'rgba(16, 185, 129, 0.2)';
    ctx.fill();
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#10b981';
    ctx.font = 'bold 9px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText("WP2: Hive Alignment (2174 Ticks)", wp2.x, wp2.y - 20);
    ctx.restore();
  } else if (currentMissionKey === 'mission_biobuzz_auto') {
    // Complete 3-Phase Auto Path
    let pLeave = inchToCanvas(-48, -30);
    ctx.save();
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    let pStart = inchToCanvas(-48, -58);
    let pFlower = inchToCanvas(-20, -30);
    let pPark = inchToCanvas(-60, -60);
    
    ctx.beginPath();
    ctx.moveTo(pStart.x, pStart.y);
    ctx.lineTo(pLeave.x, pLeave.y);
    ctx.lineTo(pFlower.x, pFlower.y);
    ctx.lineTo(pPark.x, pPark.y);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.restore();
  }

  // 8. AprilTags
  APRIL_TAGS.forEach(tag => {
    let p = inchToCanvas(tag.x, tag.y);
    let isSeen = robot.visibleTags.some(t => t.id === tag.id);

    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.fillStyle = isSeen ? '#10b981' : '#f59e0b';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.fillRect(-7, -7, 14, 14);
    ctx.strokeRect(-7, -7, 14, 14);
    
    ctx.fillStyle = '#000000';
    ctx.font = 'bold 8px monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(tag.id, 0, 0);
    ctx.restore();
  });

  // 9. Camera Field-Of-View Cone
  let robotPos = inchToCanvas(robot.x, robot.y);
  let headingRad = (robot.heading * Math.PI) / 180;
  let fovHalfRad = ((CAMERA_FOV_DEG / 2) * Math.PI) / 180;
  let maxRangePx = (CAMERA_MAX_RANGE_INCHES / FIELD_SIZE_INCHES) * canvas.width;

  ctx.save();
  ctx.translate(robotPos.x, robotPos.y);
  ctx.rotate(-headingRad + Math.PI/2);

  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.arc(0, 0, maxRangePx, -fovHalfRad - Math.PI/2, fovHalfRad - Math.PI/2);
  ctx.closePath();
  ctx.fillStyle = 'rgba(6, 182, 212, 0.12)';
  ctx.fill();
  ctx.strokeStyle = 'rgba(6, 182, 212, 0.4)';
  ctx.lineWidth = 1;
  ctx.stroke();
  ctx.restore();

  // 10. Motor Encoder Ghost Pose (Visualizing Odometry calculated strictly from motor encoders)
  let motorOdoPos = inchToCanvas(robot.motorOdoX, robot.motorOdoY);
  let motorOdoHeadingRad = (robot.motorOdoHeading * Math.PI) / 180;
  const robotWidthPx = (ROBOT_WIDTH_INCHES / FIELD_SIZE_INCHES) * canvas.width;
  const robotLengthPx = (ROBOT_LENGTH_INCHES / FIELD_SIZE_INCHES) * canvas.width;

  ctx.save();
  ctx.translate(motorOdoPos.x, motorOdoPos.y);
  ctx.rotate(-motorOdoHeadingRad + Math.PI/2);
  ctx.strokeStyle = 'rgba(245, 158, 11, 0.6)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 4]);
  ctx.strokeRect(-robotWidthPx/2, -robotLengthPx/2, robotWidthPx, robotLengthPx);
  ctx.fillStyle = 'rgba(245, 158, 11, 0.7)';
  ctx.font = 'bold 8px monospace';
  ctx.fillText('ODO POSE', -robotWidthPx/2 + 2, -robotLengthPx/2 + 10);
  ctx.setLineDash([]);
  ctx.restore();

  // 11. True Robot Model (goBILDA Strafer Chassis with 45° Mecanum Rollers & Force Vectors)
  ctx.save();
  ctx.translate(robotPos.x, robotPos.y);
  ctx.rotate(-headingRad + Math.PI/2);

  // Chassis Frame
  ctx.fillStyle = '#0f172a';
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 2;
  ctx.fillRect(-robotWidthPx/2, -robotLengthPx/2, robotWidthPx, robotLengthPx);
  ctx.strokeRect(-robotWidthPx/2, -robotLengthPx/2, robotWidthPx, robotLengthPx);

  // 4 Mecanum Wheels with 45° Rollers Pattern
  const wheelW = 6;
  const wheelL = 15;
  const wheels = [
    { x: -robotWidthPx/2 + 2, y: -robotLengthPx/2 + 8, power: robot.flPower, angle: 45, name: 'FL' },
    { x: robotWidthPx/2 - 2, y: -robotLengthPx/2 + 8, power: robot.frPower, angle: -45, name: 'FR' },
    { x: -robotWidthPx/2 + 2, y: robotLengthPx/2 - 8, power: robot.blPower, angle: -45, name: 'BL' },
    { x: robotWidthPx/2 - 2, y: robotLengthPx/2 - 8, power: robot.brPower, angle: 45, name: 'BR' }
  ];

  wheels.forEach(w => {
    ctx.save();
    ctx.translate(w.x, w.y);
    
    // Wheel Rim
    ctx.fillStyle = '#475569';
    ctx.fillRect(-wheelW/2, -wheelL/2, wheelW, wheelL);
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 0.8;
    ctx.strokeRect(-wheelW/2, -wheelL/2, wheelW, wheelL);

    // 45° Angled Rollers
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = 1;
    for (let r = -5; r <= 5; r += 3.5) {
      ctx.beginPath();
      if (w.angle === 45) {
        ctx.moveTo(-wheelW/2, r - 2);
        ctx.lineTo(wheelW/2, r + 2);
      } else {
        ctx.moveTo(-wheelW/2, r + 2);
        ctx.lineTo(wheelW/2, r - 2);
      }
      ctx.stroke();
    }

    // Force Vector Arrow on Wheel (When power != 0)
    if (Math.abs(w.power) > 0.05) {
      let vecLen = w.power * 16;
      let fAngleRad = (w.angle * Math.PI) / 180;
      let vx = Math.sin(fAngleRad) * vecLen;
      let vy = -Math.cos(fAngleRad) * vecLen;

      ctx.strokeStyle = w.power > 0 ? '#10b981' : '#f43f5e';
      ctx.fillStyle = w.power > 0 ? '#10b981' : '#f43f5e';
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(vx, vy);
      ctx.stroke();

      // Arrowhead
      let headlen = 4;
      let angle = Math.atan2(vy, vx);
      ctx.beginPath();
      ctx.moveTo(vx, vy);
      ctx.lineTo(vx - headlen * Math.cos(angle - Math.PI / 6), vy - headlen * Math.sin(angle - Math.PI / 6));
      ctx.lineTo(vx - headlen * Math.cos(angle + Math.PI / 6), vy - headlen * Math.sin(angle + Math.PI / 6));
      ctx.closePath();
      ctx.fill();
    }

    ctx.restore();
  });

  // Net Resultant Drive & Strafe Force Vector (Robot Center)
  let netDrive = (robot.flPower + robot.frPower + robot.blPower + robot.brPower) / 4;
  let netStrafe = (robot.flPower - robot.frPower - robot.blPower + robot.brPower) / 4;

  if (Math.abs(netDrive) > 0.05 || Math.abs(netStrafe) > 0.05) {
    let nx = netStrafe * 24;
    let ny = -netDrive * 24;

    ctx.strokeStyle = '#f59e0b';
    ctx.fillStyle = '#f59e0b';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(nx, ny);
    ctx.stroke();

    let hlen = 5;
    let nAngle = Math.atan2(ny, nx);
    ctx.beginPath();
    ctx.moveTo(nx, ny);
    ctx.lineTo(nx - hlen * Math.cos(nAngle - Math.PI / 6), ny - hlen * Math.sin(nAngle - Math.PI / 6));
    ctx.lineTo(nx - hlen * Math.cos(nAngle + Math.PI / 6), ny - hlen * Math.sin(nAngle + Math.PI / 6));
    ctx.closePath();
    ctx.fill();
  }

  // Front Heading Indicator
  ctx.fillStyle = '#38bdf8';
  ctx.beginPath();
  ctx.moveTo(0, -robotLengthPx/2 - 6);
  ctx.lineTo(6, -robotLengthPx/2 + 2);
  ctx.lineTo(-6, -robotLengthPx/2 + 2);
  ctx.closePath();
  ctx.fill();

  ctx.restore();
}

// -------------------------------------------------------------
// 5. OpMode Execution Engine
// -------------------------------------------------------------
function startOpMode() {
  if (opModeRunning) return;
  
  try {
    const generator = (typeof javascript !== 'undefined' && javascript.javascriptGenerator)
      ? javascript.javascriptGenerator
      : (typeof Blockly !== 'undefined' && (Blockly.JavaScript || Blockly.javascriptGenerator));

    if (!generator) {
      throw new Error("Blockly JavaScript generator not available.");
    }

    const code = generator.workspaceToCode(workspace);
    console.log("Generated FTC OpMode Code:\n", code);

    opModeCodeFunction = new Function(code);
    opModeRunning = true;
    simSetState("STATE_START");

    const runBtn = document.getElementById('runBtn');
    const stopBtn = document.getElementById('stopBtn');
    if (runBtn) runBtn.classList.add('opacity-50', 'cursor-not-allowed');
    if (stopBtn) stopBtn.disabled = false;

    // Loop rate ~20Hz (50ms)
    opModeExecutionInterval = setInterval(() => {
      if (!opModeRunning) return;
      try {
        opModeCodeFunction();
      } catch (err) {
        console.error("OpMode Runtime Error:", err);
        simTelemetryAddData("RUNTIME_ERROR", err.message);
        simTelemetryUpdate();
        stopOpMode();
      }
    }, 50);

  } catch (err) {
    alert("Error compiling Blockly code: " + err.message);
  }
}

function stopOpMode() {
  opModeRunning = false;
  if (opModeExecutionInterval) {
    clearInterval(opModeExecutionInterval);
    opModeExecutionInterval = null;
  }
  simStopRobot();
  const runBtn = document.getElementById('runBtn');
  const stopBtn = document.getElementById('stopBtn');
  if (runBtn) runBtn.classList.remove('opacity-50', 'cursor-not-allowed');
  if (stopBtn) stopBtn.disabled = true;
  simTelemetryAddData("STATUS", "OpMode Stopped");
  simTelemetryUpdate();
}

function resetRobot() {
  stopOpMode();
  robot.x = -48;
  robot.y = -58;
  robot.startX = -48;
  robot.startY = -58;
  robot.heading = 90;
  robot.flPower = 0;
  robot.frPower = 0;
  robot.blPower = 0;
  robot.brPower = 0;
  robot.motorTicksFL = 0;
  robot.motorTicksFR = 0;
  robot.motorTicksBL = 0;
  robot.motorTicksBR = 0;
  robot.motorTicks = 0;
  robot.motorOdoX = -48;
  robot.motorOdoY = -58;
  robot.motorOdoHeading = 90;
  robot.odoX = -48;
  robot.odoY = -58;
  robot.odoHeading = 90;
  robot.currentState = "STATE_IDLE";
  robot.autoLeaveScored = false;
  robot.autoParkScored = false;
  robot.autoScore = 0;
  telemetryBuffer = {};
  simTelemetryUpdate();
  
  const stateBadge = document.getElementById('activeStateBadge');
  if (stateBadge) stateBadge.innerText = "STATE_IDLE";
  const timerBadge = document.getElementById('stateTimerBadge');
  if (timerBadge) timerBadge.innerText = "0.00s";
  const scoreDisplay = document.getElementById('autoScoreDisplay');
  if (scoreDisplay) scoreDisplay.innerText = "0";
}

// -------------------------------------------------------------
// 6. Guided BioBuzz Missions & Curriculum
// -------------------------------------------------------------
const MISSIONS = {
  mission_strafing: {
    badge: "PROJECT 1: STRAFING",
    title: "🦀 Project 1: Mecanum Strafing & Vector Kinematics",
    desc: "Master the mechanics and mathematics of Mecanum sideways translation (strafing). Understand how 45° angled rollers cancel longitudinal forces while doubling lateral force vectors, and program a flawless lateral alignment routine without rotating your heading.",
    target: "Start at (X: -48\", Y: -58\") with Heading: 90° (facing North). Strafe East 28\" to align with Flower Station 1 at (X: -20\", Y: -48\"), then strafe West back to base at (X: -60\", Y: -48\") maintaining pure 90° heading throughout.",
    mathFormula: "Strafe Right: FL = +1.0, FR = -1.0, BL = -1.0, BR = +1.0\nStrafe Left:  FL = -1.0, FR = +1.0, BL = +1.0, BR = -1.0\nNet Lateral Force: F_x = (FL - FR - BL + BR) / 4",
    steps: [
      {
        step: 1,
        title: "The Physics of 45° Mecanum Rollers",
        action: "Observe the 45° roller orientation on the robot canvas. Note how FL and BR rollers point in one diagonal direction, while FR and BL rollers point in the opposite diagonal.",
        why: "<strong>Why are we doing this?</strong> Standard rubber wheels can only exert force along their plane of rotation. Mecanum wheels have free-spinning passive rollers mounted at 45°. Spinning the wheel pushes the ground along a 45° diagonal vector with both a forward component (Fy) and a sideways component (Fx)."
      },
      {
        step: 2,
        title: "Formulate the 4-Motor Strafe Power Matrix",
        action: "To strafe pure Right: Set Front Left (+0.5), Front Right (-0.5), Back Left (-0.5), and Back Right (+0.5).",
        why: "<strong>Why are we doing this?</strong> When FL(+0.5) and FR(-0.5) spin in opposite directions, their forward force components (Fy) perfectly cancel out (+0.5 - 0.5 = 0), while their lateral force components (Fx) point in the same direction and add together (+0.5 - (-0.5) = +1.0)! This creates 100% pure sideways translation without changing your heading."
      },
      {
        step: 3,
        title: "Test with Cartesian Mecanum Drive Block",
        action: "Use '🏎️ Mecanum Drive' with (Forward: 0, Strafe: 0.6, Turn: 0) to strafe laterally toward Waypoint 1 (X: -20\").",
        why: "<strong>Why are we doing this?</strong> The Cartesian drive block automatically performs this 4-wheel matrix multiplication under the hood, allowing drivers and autonomous state machines to command intuitive (vx, vy, vTheta) field vectors."
      },
      {
        step: 4,
        title: "Observe Real-Time Wheel Vectors on Canvas",
        action: "Look at the green/red vector arrows protruding from each of the 4 wheels while strafing.",
        why: "<strong>Why are we doing this?</strong> Visualizing the roller force vectors confirms in real time that opposing diagonal forces are canceling longitudinal momentum."
      },
      {
        step: 5,
        title: "Reverse Strafe Direction Back to Loading Zone",
        action: "When Odometry X >= -20, switch state to 'STATE_STRAFE_WEST' and set Strafe: -0.6 until X <= -60.",
        why: "<strong>Why are we doing this?</strong> Reversing the sign of the strafe command flips the rotational direction of all 4 wheels, producing pure leftward force into the Loading Zone."
      }
    ],
    keyBlocks: [
      "🏎️ Mecanum Drive (Cartesian)",
      "⚙️ Set 4 Motor Powers (FL, FR, BL, BR)",
      "📍 Motor Encoder Odometry X",
      "⚙️ Switch State to",
      "🛑 Stop All Motors"
    ],
    hint: "Notice how your robot never had to turn around to return to the Loading Zone! This saves crucial seconds during the 30-second autonomous period."
  },

  mission_encoder_odometry: {
    badge: "PROJECT 2: ENCODER ODOMETRY",
    title: "🎯 Project 2: Motor Encoder Odometry (Ticks to Inches)",
    desc: "Transform built-in motor encoders into a high-precision odometry tracking system. Calculate quadrature encoder pulse counts from gear ratios and wheel circumference, and navigate autonomously to field waypoints using tick thresholds rather than unreliable sleep() timers.",
    target: "Drive 36.0\" North to Waypoint 1 (Y: -22\"), then strafe 48.0\" East to the Central Hive (X: 0\") using motor encoder target thresholds.",
    mathFormula: "Ticks = Target Inches × (537.7 ticks/rev / (π × 3.7795\" wheel)) = Inches × 45.285 ticks/in\n36 inches = 1,630 ticks | 48 inches = 2,174 ticks",
    steps: [
      {
        step: 1,
        title: "Zero Out Relative Encoders",
        action: "Call '🔄 Reset Motor Encoders (Zero Ticks)' in your initialization or STATE_START block.",
        why: "<strong>Why are we doing this?</strong> Motor encoders count relative quadrature pulses from power-on. Zeroing them ensures you start from a known baseline unaffected by robot placement on the tiles."
      },
      {
        step: 2,
        title: "Calculate Target Ticks from First Principles",
        action: "For 36 inches forward: 36 in × 45.285 ticks/in = <strong>1,630 ticks</strong>. For 48 inches strafe: 48 in × 45.285 ticks/in = <strong>2,174 ticks</strong>.",
        why: "<strong>Why are we doing this?</strong> With a 19.2:1 goBILDA motor (537.7 ticks/rev) and 96mm (3.78\") wheels (Circumference = 11.874\"), each shaft revolution equals 11.874 inches. Dividing 537.7 / 11.874 gives exactly 45.285 ticks per inch."
      },
      {
        step: 3,
        title: "Drive Forward & Monitor Average Ticks",
        action: "Set Mecanum Drive (Forward: 0.5, Strafe: 0, Turn: 0). Check if 'Average Motor Encoder Ticks >= 1630'.",
        why: "<strong>Why are we doing this?</strong> Averaging all 4 motor encoders filters out individual wheel noise and micro-slips."
      },
      {
        step: 4,
        title: "Isolate Waypoint Intervals with Encoder Resets",
        action: "When 1630 ticks are reached, stop motors, reset encoders, and transition state to 'STATE_STRAFE_HIVE'.",
        why: "<strong>Why are we doing this?</strong> Re-zeroing encoders at each state transition prevents cumulative math errors across multiple autonomous legs."
      },
      {
        step: 5,
        title: "Strafe to Central Hive using 2174 Ticks",
        action: "In STATE_STRAFE_HIVE, set Mecanum Drive (Forward: 0, Strafe: 0.5, Turn: 0) until ticks >= 2174.",
        why: "<strong>Why are we doing this?</strong> Tests lateral displacement odometry under active Mecanum strafing."
      }
    ],
    keyBlocks: [
      "🔄 Reset Motor Encoders",
      "⚡ Motor Encoder Ticks (Avg)",
      "🔢 Convert Inches to Ticks",
      "🏎️ Mecanum Drive",
      "⚙️ Switch State to",
      "🛑 Stop All Motors"
    ],
    hint: "Encoder odometry provides deterministic, repeatable movement regardless of whether your battery is fully charged at 14V or depleted to 12V!"
  },

  mission_biobuzz_auto: {
    badge: "PROJECT 3: BIOBUZZ AUTO",
    title: "🏆 Project 3: Official BIOBUZZ Autonomous Routine (Leave + Park)",
    desc: "Build a complete, competition-legal 3-phase autonomous state machine for the 2026-2027 BIOBUZZ game! Earn maximum foundational autonomous match points: 3 pts for Auto Leave + 5 pts for Auto Park in the Loading Zone.",
    target: "Achieve all 8 Auto Points: 1) Disconnect from perimeter wall (Leave = 3 pts), 2) Strafe 28\" to align with Flower Station 1, 3) Return and Park inside the Loading Zone (Park = 5 pts).",
    mathFormula: "Auto Score = (Leave >= 8\" : +3 pts) + (Park in Loading Zone : +5 pts) = 8 Points",
    steps: [
      {
        step: 1,
        title: "Phase 1: Leave Wall Objective (3 Points)",
        action: "In STATE_START: Drive forward (Forward: 0.5) until Odometry Y >= -36. Switch to 'STATE_STRAFE_FLOWER'.",
        why: "<strong>Why are we doing this?</strong> Rule R102 / Table 10-2 awards 3 points to every robot that completely disconnects from the starting perimeter wall during Autonomous."
      },
      {
        step: 2,
        title: "Phase 2: Strafe to Flower 1 Alignment",
        action: "In STATE_STRAFE_FLOWER: Strafe right (Strafe: 0.5) until Odometry X >= -20. Switch to 'STATE_PARK_ZONE'.",
        why: "<strong>Why are we doing this?</strong> Positions the intake mechanism directly in line with Flower Station 1 to retrieve Pollen balls in subsequent match routines."
      },
      {
        step: 3,
        title: "Phase 3: Auto Park in Loading Zone (5 Points)",
        action: "In STATE_PARK_ZONE: Strafe left (Strafe: -0.6) and back (Forward: -0.4) into the Loading Zone (X <= -52, Y <= -52). Stop all motors.",
        why: "<strong>Why are we doing this?</strong> Parking inside the alliance Loading Zone at the end of Autonomous awards 5 points and contributes directly to earning the Swarm Ranking Point (1 RP)!"
      },
      {
        step: 4,
        title: "Telemetry Stream & Score Verification",
        action: "Call 'telemetry.addData(\"Auto Score\", getAutoScore())' and 'telemetry.update()'.",
        why: "<strong>Why are we doing this?</strong> Validates that the Driver Station receives live confirmation of all 8 Auto points scored."
      }
    ],
    keyBlocks: [
      "⚙️ Switch State to",
      "⚙️ In State?",
      "🏎️ Mecanum Drive",
      "📍 Dead-Wheel Odo X/Y",
      "🏆 Get Auto Match Points",
      "📊 telemetry.update"
    ],
    hint: "Watch the Auto Score banner at the top right of the simulator update to 'Auto Score: 8 pts' when both objectives are achieved!"
  },

  mission_wheel_slip: {
    badge: "PROJECT 4: WHEEL SLIP",
    title: "⚖️ Project 4: Wheel Slip vs Dead-Wheel Tracking Pods",
    desc: "Observe how motor drive wheels slip on foam tiles during rapid acceleration and braking, and discover why unpowered dead-wheel odometry pods are essential for millimeter-level autonomous precision.",
    target: "Accelerate rapidly across the field and observe the divergence between Motor Encoder Odometry (yellow ghost) and Dead-Wheel Odometry.",
    steps: [
      { step: 1, title: "High Acceleration", action: "Program full motor power (1.0) for rapid start-and-stop movements.", why: "High torque breaks static friction on foam tiles, inducing wheel slip." },
      { step: 2, title: "Compare Encoders vs Dead-Wheels", action: "Stream both Motor Encoder X/Y and Dead-Wheel Odo X/Y to the telemetry console.", why: "Direct comparison reveals cumulative drift error." },
      { step: 3, title: "Ghost Pose Analysis", action: "Observe the yellow dashed ghost wireframe robot (encoder calculation) vs the true robot chassis on the field canvas.", why: "Visualizes how wheel slip misleads open-loop navigation." }
    ],
    keyBlocks: ["⚡ Motor Encoder Ticks", "📍 Dead-Wheel Odo X/Y", "📊 telemetry.addData"],
    hint: "Dead-wheel odometry pods use unpowered omni-wheels held against the floor with light springs, so they never slip under motor torque!"
  },

  mission_apriltag: {
    badge: "PROJECT 5: VISION LOCALIZATION",
    title: "👁️ Project 5: AprilTag Ground-Truth Localization",
    desc: "Use the robot's front-facing camera to detect official BIOBUZZ AprilTags on the Central Hive and perimeter walls, and relocalize odometry pose to eliminate accumulated drift.",
    target: "Drive within 60 inches of Central Hive AprilTags (IDs 0, 1, 4, 38, 39, 40) or Perimeter Tags (10-13) and relocalize robot position.",
    steps: [
      { step: 1, title: "Drive into FOV", action: "Drive robot toward the Central Hive or West Wall until an AprilTag enters the cyan camera cone.", why: "Positions tag within camera detection threshold." },
      { step: 2, title: "Detect Tag ID & Distance", action: "Use 'Is AprilTag Visible?' and 'AprilTag Distance' blocks to confirm detection.", why: "Vision pipelines require valid detection checks before pose updates." },
      { step: 3, title: "Relocalize Odometry", action: "Call 'VisionPortal: Relocalize Odometry with Tag' to snap odometry pose back to ground truth.", why: "Trigonometry from known AprilTag field coordinates resets all cumulative drift to zero." }
    ],
    keyBlocks: ["👁️ Is AprilTag Visible?", "📏 AprilTag Distance", "🎯 Relocalize with Tag"],
    hint: "Check the AprilTag Vision Feed tab in the bottom console to view real-time distance and bearing angles."
  },

  sandbox: {
    badge: "FREE DRIVE",
    title: "🛠️ Sandbox Mode: Free Experimentation",
    desc: "Open playground to create custom autonomous routines, test Mecanum vector kinematics, experiment with state machines, and practice AprilTag vision tracking.",
    target: "Experiment freely with all drivetrain, sensor, and scoring blocks.",
    steps: [
      { step: 1, title: "Explore Blocks", action: "Open any category from the left toolbox to drag blocks into the workspace.", why: "Hands-on discovery helps familiarize yourself with the FTC Blockly API." },
      { step: 2, title: "Test Drive Vectors", action: "Use Mecanum Drive (Cartesian) to test forward, strafe, and turn combinations.", why: "Mecanum drivebases allow independent translation and rotation." },
      { step: 3, title: "Inspect Sensor Tabs", action: "Switch between Driver Station Telemetry, State Machine Visualizer, and AprilTag Vision Feed tabs.", why: "Real-time telemetry diagnostics are critical during competition pit testing." }
    ],
    keyBlocks: ["🏎️ Mecanum Drive", "📊 telemetry.addData", "🧭 Dead-Wheel Odo X/Y"],
    hint: "Use telemetry.addData() to display custom variables in the Driver Station console."
  }
};

let currentMissionKey = 'mission_strafing';

function loadMission(missionKey) {
  currentMissionKey = missionKey;
  const mission = MISSIONS[missionKey] || MISSIONS.mission_strafing;
  
  const badgeEl = document.getElementById('missionBadge');
  if (badgeEl) badgeEl.innerText = mission.badge;
  
  const descEl = document.getElementById('missionDesc');
  if (descEl) descEl.innerText = mission.desc;

  const guideContent = document.getElementById('missionGuideContent');
  if (guideContent) {
    let stepsHtml = mission.steps.map(s => `
      <div class="bg-slate-950/80 border border-slate-800 rounded-lg p-3 space-y-2">
        <div class="flex items-center justify-between">
          <div class="flex items-center space-x-2">
            <span class="w-5 h-5 rounded-full bg-amber-500 text-slate-950 font-bold flex items-center justify-center text-[10px]">
              ${s.step}
            </span>
            <span class="font-bold text-amber-300 text-xs">${s.title}</span>
          </div>
          <span class="text-[9px] bg-slate-800 text-slate-400 px-1.5 py-0.5 rounded font-mono">ACTION</span>
        </div>
        <p class="text-slate-200 text-xs">${s.action}</p>
        <div class="bg-slate-900/90 border-l-2 border-indigo-500 p-2 rounded text-[11px] text-indigo-200">
          ${s.why}
        </div>
      </div>
    `).join('');

    let blocksHtml = mission.keyBlocks.map(b => `<span class="bg-slate-800 text-amber-300 border border-slate-700 px-2 py-0.5 rounded text-[11px] font-mono">${b}</span>`).join(' ');

    let mathSection = mission.mathFormula ? `
      <div class="bg-amber-950/30 border border-amber-500/30 p-2.5 rounded-lg space-y-1">
        <div class="text-amber-400 font-bold uppercase tracking-wider text-[10px] flex items-center space-x-1.5">
          <i class="fa-solid fa-calculator"></i>
          <span>Kinematic / Odometry Math Formula</span>
        </div>
        <pre class="font-mono text-xs text-amber-200 bg-slate-950 p-2 rounded border border-amber-500/20 whitespace-pre-wrap">${mission.mathFormula}</pre>
      </div>
    ` : '';

    guideContent.innerHTML = `
      <div class="bg-slate-950 p-3 rounded-lg border border-slate-800 space-y-1.5">
        <div class="text-amber-400 font-bold text-sm">${mission.title}</div>
        <p class="text-slate-300 text-xs">${mission.desc}</p>
      </div>

      <div class="space-y-1.5">
        <div class="text-slate-400 font-semibold uppercase tracking-wider text-[10px]">🎯 Target Condition</div>
        <div class="bg-indigo-950/40 border border-indigo-500/30 p-2.5 rounded text-indigo-200 font-medium text-xs leading-relaxed">
          ${mission.target}
        </div>
      </div>

      ${mathSection}

      <div class="space-y-2">
        <div class="text-slate-400 font-semibold uppercase tracking-wider text-[10px]">📋 Step-by-Step Instructions & Rationale</div>
        <div class="space-y-2.5">
          ${stepsHtml}
        </div>
      </div>

      <div class="space-y-1.5">
        <div class="text-slate-400 font-semibold uppercase tracking-wider text-[10px]">🧩 Key Recommended Blocks</div>
        <div class="flex flex-wrap gap-1.5">
          ${blocksHtml}
        </div>
      </div>

      <div class="bg-amber-500/10 border border-amber-500/20 p-2.5 rounded text-amber-300 text-xs">
        💡 <strong>Coach Tip:</strong> ${mission.hint}
      </div>
    `;
  }
}

function loadStarterTemplate() {
  if (!workspace) return;
  workspace.clear();
  
  try {
    let xmlText = '';
    
    if (currentMissionKey === 'mission_strafing') {
      // Project 1 Starter: Strafing Kinematics (Forward, Strafe East to Flower 1, Strafe West to Base)
      xmlText = `<xml xmlns="https://developers.google.com/blockly/xml">
  <block type="controls_if" x="30" y="30">
    <mutation elseif="2"></mutation>
    <value name="IF0">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_START</field></block>
        </value>
      </block>
    </value>
    <statement name="DO0">
      <block type="ftc_reset_motor_encoders">
        <next>
          <block type="ftc_set_state">
            <value name="STATE">
              <block type="text"><field name="TEXT">STATE_STRAFE_EAST</field></block>
            </value>
          </block>
        </next>
      </block>
    </statement>
    <value name="IF1">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_STRAFE_EAST</field></block>
        </value>
      </block>
    </value>
    <statement name="DO1">
      <block type="ftc_mecanum_drive">
        <value name="DRIVE"><block type="math_number"><field name="NUM">0</field></block></value>
        <value name="STRAFE"><block type="math_number"><field name="NUM">0.6</field></block></value>
        <value name="TURN"><block type="math_number"><field name="NUM">0</field></block></value>
        <next>
          <block type="ftc_telemetry_add_data">
            <value name="CAPTION"><block type="text"><field name="TEXT">Phase</field></block></value>
            <value name="VALUE"><block type="text"><field name="TEXT">Strafing East to Flower 1</field></block></value>
            <next>
              <block type="ftc_telemetry_add_data">
                <value name="CAPTION"><block type="text"><field name="TEXT">Odometry X</field></block></value>
                <value name="VALUE"><block type="ftc_get_odo_x"></block></value>
                <next>
                  <block type="controls_if">
                    <value name="IF0">
                      <block type="logic_compare">
                        <field name="OP">GTE</field>
                        <value name="A"><block type="ftc_get_odo_x"></block></value>
                        <value name="B"><block type="math_number"><field name="NUM">-20</field></block></value>
                      </block>
                    </value>
                    <statement name="DO0">
                      <block type="ftc_stop_robot">
                        <next>
                          <block type="ftc_set_state">
                            <value name="STATE">
                              <block type="text"><field name="TEXT">STATE_STRAFE_WEST</field></block>
                            </value>
                          </block>
                        </next>
                      </block>
                    </statement>
                  </block>
                </next>
              </block>
            </next>
          </block>
        </next>
      </block>
    </statement>
    <value name="IF2">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_STRAFE_WEST</field></block>
        </value>
      </block>
    </value>
    <statement name="DO2">
      <block type="ftc_mecanum_drive">
        <value name="DRIVE"><block type="math_number"><field name="NUM">0</field></block></value>
        <value name="STRAFE"><block type="math_number"><field name="NUM">-0.6</field></block></value>
        <value name="TURN"><block type="math_number"><field name="NUM">0</field></block></value>
        <next>
          <block type="ftc_telemetry_add_data">
            <value name="CAPTION"><block type="text"><field name="TEXT">Phase</field></block></value>
            <value name="VALUE"><block type="text"><field name="TEXT">Strafing West to Base</field></block></value>
            <next>
              <block type="controls_if">
                <value name="IF0">
                  <block type="logic_compare">
                    <field name="OP">LTE</field>
                    <value name="A"><block type="ftc_get_odo_x"></block></value>
                    <value name="B"><block type="math_number"><field name="NUM">-60</field></block></value>
                  </block>
                </value>
                <statement name="DO0">
                  <block type="ftc_stop_robot">
                    <next>
                      <block type="ftc_set_state">
                        <value name="STATE">
                          <block type="text"><field name="TEXT">STATE_COMPLETE</field></block>
                        </value>
                      </block>
                    </next>
                  </block>
                </statement>
              </block>
            </next>
          </block>
        </next>
      </block>
    </statement>
    <next>
      <block type="ftc_telemetry_update"></block>
    </next>
  </block>
</xml>`;
    } else if (currentMissionKey === 'mission_encoder_odometry') {
      // Project 2 Starter: Motor Encoder Odometry (36" forward = 1630 ticks, 48" strafe = 2174 ticks)
      xmlText = `<xml xmlns="https://developers.google.com/blockly/xml">
  <block type="controls_if" x="30" y="30">
    <mutation elseif="2"></mutation>
    <value name="IF0">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_START</field></block>
        </value>
      </block>
    </value>
    <statement name="DO0">
      <block type="ftc_reset_motor_encoders">
        <next>
          <block type="ftc_set_state">
            <value name="STATE">
              <block type="text"><field name="TEXT">STATE_DRIVE_FORWARD</field></block>
            </value>
          </block>
        </next>
      </block>
    </statement>
    <value name="IF1">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_DRIVE_FORWARD</field></block>
        </value>
      </block>
    </value>
    <statement name="DO1">
      <block type="ftc_mecanum_drive">
        <value name="DRIVE"><block type="math_number"><field name="NUM">0.5</field></block></value>
        <value name="STRAFE"><block type="math_number"><field name="NUM">0</field></block></value>
        <value name="TURN"><block type="math_number"><field name="NUM">0</field></block></value>
        <next>
          <block type="ftc_telemetry_add_data">
            <value name="CAPTION"><block type="text"><field name="TEXT">Phase</field></block></value>
            <value name="VALUE"><block type="text"><field name="TEXT">Driving 36in North (1630 Ticks)</field></block></value>
            <next>
              <block type="ftc_telemetry_add_data">
                <value name="CAPTION"><block type="text"><field name="TEXT">Current Ticks</field></block></value>
                <value name="VALUE"><block type="ftc_get_wheel_ticks"></block></value>
                <next>
                  <block type="controls_if">
                    <value name="IF0">
                      <block type="logic_compare">
                        <field name="OP">GTE</field>
                        <value name="A"><block type="ftc_get_wheel_ticks"></block></value>
                        <value name="B"><block type="math_number"><field name="NUM">1630</field></block></value>
                      </block>
                    </value>
                    <statement name="DO0">
                      <block type="ftc_stop_robot">
                        <next>
                          <block type="ftc_reset_motor_encoders">
                            <next>
                              <block type="ftc_set_state">
                                <value name="STATE">
                                  <block type="text"><field name="TEXT">STATE_STRAFE_RIGHT</field></block>
                                </value>
                              </block>
                            </next>
                          </block>
                        </next>
                      </block>
                    </statement>
                  </block>
                </next>
              </block>
            </next>
          </block>
        </next>
      </block>
    </statement>
    <value name="IF2">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_STRAFE_RIGHT</field></block>
        </value>
      </block>
    </value>
    <statement name="DO2">
      <block type="ftc_mecanum_drive">
        <value name="DRIVE"><block type="math_number"><field name="NUM">0</field></block></value>
        <value name="STRAFE"><block type="math_number"><field name="NUM">0.5</field></block></value>
        <value name="TURN"><block type="math_number"><field name="NUM">0</field></block></value>
        <next>
          <block type="ftc_telemetry_add_data">
            <value name="CAPTION"><block type="text"><field name="TEXT">Phase</field></block></value>
            <value name="VALUE"><block type="text"><field name="TEXT">Strafing 48in to Hive (2174 Ticks)</field></block></value>
            <next>
              <block type="controls_if">
                <value name="IF0">
                  <block type="logic_compare">
                    <field name="OP">GTE</field>
                    <value name="A"><block type="ftc_get_wheel_ticks"></block></value>
                    <value name="B"><block type="math_number"><field name="NUM">2174</field></block></value>
                  </block>
                </value>
                <statement name="DO0">
                  <block type="ftc_stop_robot">
                    <next>
                      <block type="ftc_set_state">
                        <value name="STATE">
                          <block type="text"><field name="TEXT">STATE_COMPLETE</field></block>
                        </value>
                      </block>
                    </next>
                  </block>
                </statement>
              </block>
            </next>
          </block>
        </next>
      </block>
    </statement>
    <next>
      <block type="ftc_telemetry_update"></block>
    </next>
  </block>
</xml>`;
    } else if (currentMissionKey === 'mission_biobuzz_auto') {
      // Project 3 Starter: Official BIOBUZZ Autonomous Routine (Leave + Strafe + Park = 8 pts)
      xmlText = `<xml xmlns="https://developers.google.com/blockly/xml">
  <block type="controls_if" x="30" y="30">
    <mutation elseif="2"></mutation>
    <value name="IF0">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_START</field></block>
        </value>
      </block>
    </value>
    <statement name="DO0">
      <block type="ftc_mecanum_drive">
        <value name="DRIVE"><block type="math_number"><field name="NUM">0.5</field></block></value>
        <value name="STRAFE"><block type="math_number"><field name="NUM">0</field></block></value>
        <value name="TURN"><block type="math_number"><field name="NUM">0</field></block></value>
        <next>
          <block type="ftc_telemetry_add_data">
            <value name="CAPTION"><block type="text"><field name="TEXT">Auto State</field></block></value>
            <value name="VALUE"><block type="text"><field name="TEXT">1. Leave Wall (+3 Pts)</field></block></value>
            <next>
              <block type="controls_if">
                <value name="IF0">
                  <block type="logic_compare">
                    <field name="OP">GTE</field>
                    <value name="A"><block type="ftc_get_odo_y"></block></value>
                    <value name="B"><block type="math_number"><field name="NUM">-30</field></block></value>
                  </block>
                </value>
                <statement name="DO0">
                  <block type="ftc_set_state">
                    <value name="STATE"><block type="text"><field name="TEXT">STATE_STRAFE_FLOWER</field></block></value>
                  </block>
                </statement>
              </block>
            </next>
          </block>
        </next>
      </block>
    </statement>
    <value name="IF1">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_STRAFE_FLOWER</field></block>
        </value>
      </block>
    </value>
    <statement name="DO1">
      <block type="ftc_mecanum_drive">
        <value name="DRIVE"><block type="math_number"><field name="NUM">0</field></block></value>
        <value name="STRAFE"><block type="math_number"><field name="NUM">0.5</field></block></value>
        <value name="TURN"><block type="math_number"><field name="NUM">0</field></block></value>
        <next>
          <block type="ftc_telemetry_add_data">
            <value name="CAPTION"><block type="text"><field name="TEXT">Auto State</field></block></value>
            <value name="VALUE"><block type="text"><field name="TEXT">2. Strafe to Flower 1</field></block></value>
            <next>
              <block type="controls_if">
                <value name="IF0">
                  <block type="logic_compare">
                    <field name="OP">GTE</field>
                    <value name="A"><block type="ftc_get_odo_x"></block></value>
                    <value name="B"><block type="math_number"><field name="NUM">-20</field></block></value>
                  </block>
                </value>
                <statement name="DO0">
                  <block type="ftc_set_state">
                    <value name="STATE"><block type="text"><field name="TEXT">STATE_AUTO_PARK</field></block></value>
                  </block>
                </statement>
              </block>
            </next>
          </block>
        </next>
      </block>
    </statement>
    <value name="IF2">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_AUTO_PARK</field></block>
        </value>
      </block>
    </value>
    <statement name="DO2">
      <block type="ftc_mecanum_drive">
        <value name="DRIVE"><block type="math_number"><field name="NUM">-0.4</field></block></value>
        <value name="STRAFE"><block type="math_number"><field name="NUM">-0.6</field></block></value>
        <value name="TURN"><block type="math_number"><field name="NUM">0</field></block></value>
        <next>
          <block type="ftc_telemetry_add_data">
            <value name="CAPTION"><block type="text"><field name="TEXT">Auto State</field></block></value>
            <value name="VALUE"><block type="text"><field name="TEXT">3. Parking in Zone (+5 Pts)</field></block></value>
            <next>
              <block type="controls_if">
                <value name="IF0">
                  <block type="logic_operation">
                    <field name="OP">AND</field>
                    <value name="A">
                      <block type="logic_compare">
                        <field name="OP">LTE</field>
                        <value name="A"><block type="ftc_get_odo_x"></block></value>
                        <value name="B"><block type="math_number"><field name="NUM">-52</field></block></value>
                      </block>
                    </value>
                    <value name="B">
                      <block type="logic_compare">
                        <field name="OP">LTE</field>
                        <value name="A"><block type="ftc_get_odo_y"></block></value>
                        <value name="B"><block type="math_number"><field name="NUM">-52</field></block></value>
                      </block>
                    </value>
                  </block>
                </value>
                <statement name="DO0">
                  <block type="ftc_stop_robot">
                    <next>
                      <block type="ftc_set_state">
                        <value name="STATE"><block type="text"><field name="TEXT">STATE_COMPLETE</field></block></value>
                      </block>
                    </next>
                  </block>
                </statement>
              </block>
            </next>
          </block>
        </next>
      </block>
    </statement>
    <next>
      <block type="ftc_telemetry_add_data">
        <value name="CAPTION"><block type="text"><field name="TEXT">Auto Points Earned</field></block></value>
        <value name="VALUE"><block type="ftc_get_auto_score"></block></value>
        <next>
          <block type="ftc_telemetry_update"></block>
        </next>
      </block>
    </next>
  </block>
</xml>`;
    } else {
      // General Sandbox / AprilTag / Slip Starter
      xmlText = `<xml xmlns="https://developers.google.com/blockly/xml">
  <block type="controls_if" x="30" y="30">
    <mutation elseif="1"></mutation>
    <value name="IF0">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_START</field></block>
        </value>
      </block>
    </value>
    <statement name="DO0">
      <block type="ftc_mecanum_drive">
        <value name="DRIVE"><block type="math_number"><field name="NUM">0.5</field></block></value>
        <value name="STRAFE"><block type="math_number"><field name="NUM">0</field></block></value>
        <value name="TURN"><block type="math_number"><field name="NUM">0</field></block></value>
        <next>
          <block type="ftc_telemetry_add_data">
            <value name="CAPTION"><block type="text"><field name="TEXT">State</field></block></value>
            <value name="VALUE"><block type="text"><field name="TEXT">Driving Forward</field></block></value>
            <next>
              <block type="controls_if">
                <value name="IF0">
                  <block type="logic_compare">
                    <field name="OP">GTE</field>
                    <value name="A"><block type="ftc_get_odo_y"></block></value>
                    <value name="B"><block type="math_number"><field name="NUM">0</field></block></value>
                  </block>
                </value>
                <statement name="DO0">
                  <block type="ftc_set_state">
                    <value name="STATE"><block type="text"><field name="TEXT">STATE_SCAN_TAG</field></block></value>
                  </block>
                </statement>
              </block>
            </next>
          </block>
        </next>
      </block>
    </statement>
    <value name="IF1">
      <block type="ftc_is_state">
        <value name="STATE">
          <block type="text"><field name="TEXT">STATE_SCAN_TAG</field></block>
        </value>
      </block>
    </value>
    <statement name="DO1">
      <block type="ftc_stop_robot">
        <next>
          <block type="controls_if">
            <value name="IF0">
              <block type="ftc_is_apriltag_visible">
                <value name="TAG_ID"><block type="math_number"><field name="NUM">0</field></block></value>
              </block>
            </value>
            <statement name="DO0">
              <block type="ftc_relocalize_with_apriltag">
                <value name="TAG_ID"><block type="math_number"><field name="NUM">0</field></block></value>
              </block>
            </statement>
          </block>
        </next>
      </block>
    </statement>
    <next>
      <block type="ftc_telemetry_update"></block>
    </next>
  </block>
</xml>`;
    }

    const dom = Blockly.utils.xml.textToDom(xmlText);
    Blockly.Xml.domToWorkspace(dom, workspace);
  } catch (err) {
    console.error("Error loading starter template into workspace:", err);
  }
}

// -------------------------------------------------------------
// 7. App Initialization & Event Listeners
// -------------------------------------------------------------
let workspace = null;

window.addEventListener('DOMContentLoaded', () => {
  // 1. 60FPS Physics & Canvas Rendering loop
  let lastTime = performance.now();
  function animationLoop(now) {
    let dt = (now - lastTime) / 1000.0;
    if (dt > 0.1) dt = 0.1;
    lastTime = now;

    try {
      updatePhysics(dt);
      renderField();
    } catch (renderErr) {
      console.error("Render loop error:", renderErr);
    }
    requestAnimationFrame(animationLoop);
  }
  requestAnimationFrame(animationLoop);

  // 2. Register custom FTC blocks
  try {
    registerFTCBlocks();
  } catch (blockErr) {
    console.error("Error registering FTC blocks:", blockErr);
  }

  // 3. Initialize Blockly Workspace
  try {
    workspace = Blockly.inject('blocklyDiv', {
      toolbox: document.getElementById('toolbox'),
      theme: Blockly.Themes && Blockly.Themes.Dark ? Blockly.Themes.Dark : undefined,
      grid: { spacing: 20, length: 3, colour: '#334155', snap: true },
      zoom: { controls: true, wheel: true, startScale: 0.9, maxScale: 2, minScale: 0.5, scaleSpeed: 1.15 },
      trashcan: true
    });
  } catch (injectErr) {
    console.error("Error injecting Blockly workspace:", injectErr);
  }

  // 4. Load Mission Guide and Starter Template
  try {
    loadMission('mission_strafing');
    loadStarterTemplate();
  } catch (missionErr) {
    console.error("Error loading mission/starter template:", missionErr);
  }

  // Handle resizing
  window.addEventListener('resize', () => {
    if (workspace) Blockly.svgResize(workspace);
  });
  setTimeout(() => {
    if (workspace) Blockly.svgResize(workspace);
  }, 100);

  // Button Listeners
  const runBtn = document.getElementById('runBtn');
  const stopBtn = document.getElementById('stopBtn');
  const resetRobotBtn = document.getElementById('resetRobotBtn');
  const clearWorkspaceBtn = document.getElementById('clearWorkspaceBtn');
  const loadTemplateBtn = document.getElementById('loadTemplateBtn');
  const clearTelemetryBtn = document.getElementById('clearTelemetryBtn');

  if (runBtn) runBtn.addEventListener('click', startOpMode);
  if (stopBtn) stopBtn.addEventListener('click', stopOpMode);
  if (resetRobotBtn) resetRobotBtn.addEventListener('click', resetRobot);
  if (clearWorkspaceBtn) clearWorkspaceBtn.addEventListener('click', () => workspace && workspace.clear());
  if (loadTemplateBtn) loadTemplateBtn.addEventListener('click', loadStarterTemplate);
  if (clearTelemetryBtn) clearTelemetryBtn.addEventListener('click', () => {
    telemetryBuffer = {};
    simTelemetryUpdate();
  });

  // Mission Selector Listener
  const missionSelector = document.getElementById('missionSelector');
  if (missionSelector) {
    missionSelector.addEventListener('change', (e) => {
      loadMission(e.target.value);
      loadStarterTemplate();
      const missionGuideModal = document.getElementById('missionGuideModal');
      if (missionGuideModal) {
        missionGuideModal.classList.remove('translate-x-full');
      }
    });
  }

  // Tab Switching
  const tabTelemetryBtn = document.getElementById('tabTelemetryBtn');
  const tabStateBtn = document.getElementById('tabStateBtn');
  const tabVisionBtn = document.getElementById('tabVisionBtn');

  const tabTelemetry = document.getElementById('tabTelemetry');
  const tabState = document.getElementById('tabState');
  const tabVision = document.getElementById('tabVision');

  if (tabTelemetryBtn && tabStateBtn && tabVisionBtn) {
    tabTelemetryBtn.addEventListener('click', () => {
      tabTelemetry.classList.remove('hidden'); tabState.classList.add('hidden'); tabVision.classList.add('hidden');
      tabTelemetryBtn.className = "text-amber-400 font-semibold border-b-2 border-amber-400 pb-1 flex items-center space-x-1.5";
      tabStateBtn.className = "text-slate-400 hover:text-slate-200 pb-1 flex items-center space-x-1.5";
      tabVisionBtn.className = "text-slate-400 hover:text-slate-200 pb-1 flex items-center space-x-1.5";
    });

    tabStateBtn.addEventListener('click', () => {
      tabTelemetry.classList.add('hidden'); tabState.classList.remove('hidden'); tabVision.classList.add('hidden');
      tabStateBtn.className = "text-amber-400 font-semibold border-b-2 border-amber-400 pb-1 flex items-center space-x-1.5";
      tabTelemetryBtn.className = "text-slate-400 hover:text-slate-200 pb-1 flex items-center space-x-1.5";
      tabVisionBtn.className = "text-slate-400 hover:text-slate-200 pb-1 flex items-center space-x-1.5";
    });

    tabVisionBtn.addEventListener('click', () => {
      tabTelemetry.classList.add('hidden'); tabState.classList.add('hidden'); tabVision.classList.remove('hidden');
      tabVisionBtn.className = "text-amber-400 font-semibold border-b-2 border-amber-400 pb-1 flex items-center space-x-1.5";
      tabTelemetryBtn.className = "text-slate-400 hover:text-slate-200 pb-1 flex items-center space-x-1.5";
      tabStateBtn.className = "text-slate-400 hover:text-slate-200 pb-1 flex items-center space-x-1.5";
    });
  }

  // Mission Guide Drawer
  const missionGuideBtn = document.getElementById('missionGuideBtn');
  const missionGuideModal = document.getElementById('missionGuideModal');
  const closeMissionGuideBtn = document.getElementById('closeMissionGuideBtn');
  const guideLoadTemplateBtn = document.getElementById('guideLoadTemplateBtn');

  if (missionGuideBtn) {
    missionGuideBtn.addEventListener('click', () => {
      missionGuideModal.classList.toggle('translate-x-full');
      const kataModal = document.getElementById('kataModal');
      if (kataModal) kataModal.classList.add('translate-x-full');
    });
  }

  if (closeMissionGuideBtn) {
    closeMissionGuideBtn.addEventListener('click', () => {
      missionGuideModal.classList.add('translate-x-full');
    });
  }

  if (guideLoadTemplateBtn) {
    guideLoadTemplateBtn.addEventListener('click', () => {
      loadStarterTemplate();
      missionGuideModal.classList.add('translate-x-full');
    });
  }

  // Kata Modal / Slide-out
  const kataDrawerBtn = document.getElementById('kataDrawerBtn');
  const kataModal = document.getElementById('kataModal');
  const closeKataBtn = document.getElementById('closeKataBtn');

  if (kataDrawerBtn) {
    kataDrawerBtn.addEventListener('click', () => {
      kataModal.classList.toggle('translate-x-full');
      if (missionGuideModal) missionGuideModal.classList.add('translate-x-full');
    });
  }

  if (closeKataBtn) {
    closeKataBtn.addEventListener('click', () => {
      kataModal.classList.add('translate-x-full');
    });
  }

  const copyKataBtn = document.getElementById('copyKataBtn');
  if (copyKataBtn) {
    copyKataBtn.addEventListener('click', () => {
      const target = document.getElementById('kataTarget').value;
      const actual = document.getElementById('kataActual').value;
      const obstacle = document.getElementById('kataObstacle').value;
      const experiment = document.getElementById('kataExperiment').value;
      const learning = document.getElementById('kataLearning').value;

      const formatted = `### 🥋 Kata Coaching Entry (${new Date().toLocaleDateString()})
- **1. Target Condition:** ${target || 'N/A'}
- **2. Actual Condition:** ${actual || 'N/A'}
- **3. Obstacle Addressed:** ${obstacle || 'N/A'}
- **4. Next Step / Experiment (PDCA):** ${experiment || 'N/A'}
- **5. Key Learning / Engineering Takeaway:** ${learning || 'N/A'}`;

      navigator.clipboard.writeText(formatted).then(() => {
        alert("✅ Kata entry copied to clipboard! Paste it into your Engineering Notebook or log file.");
      });
    });
  }

  const clearKataBtn = document.getElementById('clearKataBtn');
  if (clearKataBtn) {
    clearKataBtn.addEventListener('click', () => {
      document.getElementById('kataTarget').value = '';
      document.getElementById('kataActual').value = '';
      document.getElementById('kataObstacle').value = '';
      document.getElementById('kataExperiment').value = '';
      document.getElementById('kataLearning').value = '';
    });
  }
});
