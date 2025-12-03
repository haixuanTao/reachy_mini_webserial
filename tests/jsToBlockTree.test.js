const { jsToBlockTree, getAllBlockTypes } = require('../src/jsToBlockTree');

describe('JS to Block Tree Converter', () => {

  describe('Basic function calls', () => {
    test('Robot.setPositionLimited creates set_joint block', () => {
      const result = jsToBlockTree('Robot.setPositionLimited(17, 2048);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('set_joint');
      expect(result.blocks[0].fields.MOTOR).toBe(17);
    });

    test('Robot.getPosition creates get_joint block', () => {
      const result = jsToBlockTree('Robot.getPosition(17);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('get_joint');
      expect(result.blocks[0].fields.MOTOR).toBe(17);
    });

    test('logConsole creates log block', () => {
      const result = jsToBlockTree('logConsole("hello");');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('log');
    });

    test('wait creates wait block', () => {
      const result = jsToBlockTree('wait(1);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('wait');
    });

    test('sleep creates wait_ms block', () => {
      const result = jsToBlockTree('sleep(500);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('wait_ms');
    });

    test('alert creates alert block', () => {
      const result = jsToBlockTree('alert("message");');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('alert');
    });
  });

  describe('Variable declarations', () => {
    test('let with number creates variables_set with math_number', () => {
      const result = jsToBlockTree('let x = 100;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].fields.VAR).toBe('x');
      expect(result.blocks[0].values.VALUE.type).toBe('math_number');
      expect(result.blocks[0].values.VALUE.fields.NUM).toBe(100);
    });

    test('var with function call creates nested blocks', () => {
      const result = jsToBlockTree('var pos = Robot.getPosition(17);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].values.VALUE.type).toBe('get_joint');
    });

    test('let with string creates variables_set with text', () => {
      const result = jsToBlockTree('let msg = "hello";');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].values.VALUE.type).toBe('text');
      expect(result.blocks[0].values.VALUE.fields.TEXT).toBe('hello');
    });

    test('assignment expression creates variables_set', () => {
      const result = jsToBlockTree('x = x + 10;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].fields.VAR).toBe('x');
      expect(result.blocks[0].values.VALUE.type).toBe('math_arithmetic');
    });

    test('assignment with function call', () => {
      const result = jsToBlockTree('deg = Robot.getDegrees(17);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].fields.VAR).toBe('deg');
      expect(result.blocks[0].values.VALUE.type).toBe('get_degrees');
    });

    test('compound assignment += creates variables_set with math', () => {
      const result = jsToBlockTree('x += 5;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].fields.VAR).toBe('x');
      expect(result.blocks[0].values.VALUE.type).toBe('math_arithmetic');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('ADD');
    });

    test('compound assignment -= works', () => {
      const result = jsToBlockTree('x -= 10;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('MINUS');
    });

    test('array index assignment creates lists_setIndex', () => {
      const result = jsToBlockTree('arr[2] = 100;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('lists_setIndex');
      expect(result.blocks[0].values.TO.type).toBe('math_number');
    });

    test('array index assignment with variable index', () => {
      const result = jsToBlockTree('positions[i] = newValue;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('lists_setIndex');
      expect(result.blocks[0].values.AT.type).toBe('variables_get');
    });
  });

  describe('Control structures', () => {
    test('for loop creates controls_for block', () => {
      const result = jsToBlockTree('for (let i = 0; i < 5; i++) { logConsole("test"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('controls_for');
      expect(result.blocks[0].fields.VAR).toBe('i');
      expect(result.blocks[0].values.FROM.fields.NUM).toBe(0);
      expect(result.blocks[0].values.TO.fields.NUM).toBe(5);
      expect(result.blocks[0].statements.DO[0].type).toBe('log');
    });

    test('while loop creates controls_whileUntil block', () => {
      const result = jsToBlockTree('while (x < 10) { logConsole("looping"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('controls_whileUntil');
      expect(result.blocks[0].values.BOOL.type).toBe('logic_compare');
    });

    test('if statement creates controls_if block', () => {
      const result = jsToBlockTree('if (x > 10) { logConsole("big"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('controls_if');
      expect(result.blocks[0].values.IF0.type).toBe('logic_compare');
      expect(result.blocks[0].statements.DO0[0].type).toBe('log');
    });

    test('if-else statement creates controls_if with else', () => {
      const result = jsToBlockTree('if (x > 10) { logConsole("big"); } else { logConsole("small"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('controls_if');
      expect(result.blocks[0].extra.hasElse).toBe(true);
      expect(result.blocks[0].statements.DO0[0].type).toBe('log');
      expect(result.blocks[0].statements.ELSE[0].type).toBe('log');
    });

    test('if-else with multiple statements in each branch', () => {
      const code = `
        if (i % 2 == 0) {
          coords[2] = coords[2] + 1;
          logConsole("up");
        } else {
          coords[2] = coords[2] - 1;
          logConsole("down");
        }
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('controls_if');
      expect(result.blocks[0].statements.DO0.length).toBe(2);
      expect(result.blocks[0].statements.ELSE.length).toBe(2);
      expect(result.blocks[0].statements.DO0[0].type).toBe('lists_setIndex');
      expect(result.blocks[0].statements.ELSE[0].type).toBe('lists_setIndex');
    });
  });

  describe('Expressions', () => {
    test('boolean true creates logic_boolean', () => {
      const result = jsToBlockTree('if (true) { logConsole("yes"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.IF0.type).toBe('logic_boolean');
      expect(result.blocks[0].values.IF0.fields.BOOL).toBe('TRUE');
    });

    test('boolean false creates logic_boolean', () => {
      const result = jsToBlockTree('if (false) { logConsole("no"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.IF0.type).toBe('logic_boolean');
      expect(result.blocks[0].values.IF0.fields.BOOL).toBe('FALSE');
    });

    test('negation -x creates math_single NEG', () => {
      const result = jsToBlockTree('let y = -x;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_single');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('NEG');
    });

    test('logical not !x creates logic_negate', () => {
      const result = jsToBlockTree('if (!done) { logConsole("running"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.IF0.type).toBe('logic_negate');
    });

    test('logical AND creates logic_operation', () => {
      const result = jsToBlockTree('if (a && b) { logConsole("both"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.IF0.type).toBe('logic_operation');
      expect(result.blocks[0].values.IF0.fields.OP).toBe('AND');
    });

    test('logical OR creates logic_operation', () => {
      const result = jsToBlockTree('if (a || b) { logConsole("either"); }');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.IF0.type).toBe('logic_operation');
      expect(result.blocks[0].values.IF0.fields.OP).toBe('OR');
    });

    test('modulo operator creates math_arithmetic MODULO', () => {
      const result = jsToBlockTree('let r = x % 2;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_arithmetic');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('MODULO');
    });

    test('Math.floor creates math_single ROUNDDOWN', () => {
      const result = jsToBlockTree('let y = Math.floor(x);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_single');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('ROUNDDOWN');
    });

    test('Math.ceil creates math_single ROUNDUP', () => {
      const result = jsToBlockTree('let y = Math.ceil(x);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_single');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('ROUNDUP');
    });

    test('Math.round creates math_single ROUND', () => {
      const result = jsToBlockTree('let y = Math.round(x);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_single');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('ROUND');
    });

    test('Math.abs creates math_single ABS', () => {
      const result = jsToBlockTree('let y = Math.abs(x);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_single');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('ABS');
    });

    test('Math.sqrt creates math_single ROOT', () => {
      const result = jsToBlockTree('let y = Math.sqrt(x);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_single');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('ROOT');
    });

    test('Math.floor with expression', () => {
      const result = jsToBlockTree('var position18 = Math.floor((torque17 / 1023) * 4095);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_single');
      expect(result.blocks[0].values.VALUE.values.NUM.type).toBe('math_arithmetic');
    });

    test('comparison operators map correctly', () => {
      const ops = [
        { code: 'x < 5', expected: 'LT' },
        { code: 'x > 5', expected: 'GT' },
        { code: 'x <= 5', expected: 'LTE' },
        { code: 'x >= 5', expected: 'GTE' },
        { code: 'x == 5', expected: 'EQ' },
        { code: 'x === 5', expected: 'EQ' },
        { code: 'x != 5', expected: 'NEQ' },
        { code: 'x !== 5', expected: 'NEQ' }
      ];

      ops.forEach(({ code, expected }) => {
        const result = jsToBlockTree(`if (${code}) { logConsole("test"); }`);
        expect(result.success).toBe(true);
        expect(result.blocks[0].values.IF0.fields.OP).toBe(expected);
      });
    });

    test('math operators create math_arithmetic blocks', () => {
      const result = jsToBlockTree('let result = 5 + 3;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('math_arithmetic');
      expect(result.blocks[0].values.VALUE.fields.OP).toBe('ADD');
    });

    test('string concatenation creates text_join block', () => {
      const result = jsToBlockTree('logConsole("Value: " + x);');
      expect(result.success).toBe(true);
      const msgValue = result.blocks[0].values.MESSAGE;
      expect(msgValue.type).toBe('text_join');
      expect(msgValue.extra.itemCount).toBe(2);
    });

    test('multi-part string concatenation flattens correctly', () => {
      const result = jsToBlockTree('logConsole("Motor " + i + " pos: " + pos);');
      expect(result.success).toBe(true);
      const msgValue = result.blocks[0].values.MESSAGE;
      expect(msgValue.type).toBe('text_join');
      expect(msgValue.extra.itemCount).toBe(4);
    });
  });

  describe('Arrays', () => {
    test('array literal creates lists_create_with block', () => {
      const result = jsToBlockTree('let arr = [1, 2, 3];');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('lists_create_with');
      expect(result.blocks[0].values.VALUE.extra.itemCount).toBe(3);
    });

    test('array.length creates lists_length block', () => {
      const result = jsToBlockTree('let len = arr.length;');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('lists_length');
    });

    test('array index access creates lists_getIndex block', () => {
      const result = jsToBlockTree('let val = arr[0];');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('lists_getIndex');
    });

    test('array index with variable creates lists_getIndex block', () => {
      const result = jsToBlockTree('let val = arr[i];');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('lists_getIndex');
      expect(result.blocks[0].values.VALUE.values.AT.type).toBe('variables_get');
    });
  });

  describe('Kinematics functions', () => {
    test('Robot.getAllPositions creates get_all_joints block', () => {
      const result = jsToBlockTree('let positions = Robot.getAllPositions();');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('get_all_joints');
    });

    test('Robot.jointsToDegrees creates joints_to_degrees block', () => {
      const result = jsToBlockTree('let degrees = Robot.jointsToDegrees(positions);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('joints_to_degrees');
    });

    test('Robot.degreesToJoints creates degrees_to_joints block', () => {
      const result = jsToBlockTree('let joints = Robot.degreesToJoints(degrees);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('degrees_to_joints');
    });

    test('Robot.jointsToCoordinates creates joints_to_coordinates block', () => {
      const result = jsToBlockTree('let coords = Robot.jointsToCoordinates(joints);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('joints_to_coordinates');
    });

    test('Robot.coordinatesToJoints creates coordinates_to_joints block', () => {
      const result = jsToBlockTree('let joints = Robot.coordinatesToJoints(coords);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('coordinates_to_joints');
    });

    test('Robot.getCurrentCoordinates creates get_current_coordinates block', () => {
      const result = jsToBlockTree('let coords = Robot.getCurrentCoordinates();');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('get_current_coordinates');
    });
  });

  describe('Motor control functions', () => {
    test('Robot.setTorque(motor, true) creates enable_torque block', () => {
      const result = jsToBlockTree('Robot.setTorque(17, true);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('enable_torque');
    });

    test('Robot.setTorque(motor, false) creates disable_torque block', () => {
      const result = jsToBlockTree('Robot.setTorque(17, false);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('disable_torque');
    });

    test('Robot.moveSmooth creates move_smooth block', () => {
      const result = jsToBlockTree('Robot.moveSmooth(17, 2048, 1000);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('move_smooth');
      expect(result.blocks[0].fields.MOTOR).toBe(17);
    });

    test('Robot.setSpeed creates set_speed block', () => {
      const result = jsToBlockTree('Robot.setSpeed(17, 100);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].type).toBe('set_speed');
    });

    test('Robot.isMoving creates is_moving block', () => {
      const result = jsToBlockTree('let moving = Robot.isMoving(17);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('is_moving');
    });

    test('Robot.getLoad creates get_load block', () => {
      const result = jsToBlockTree('let load = Robot.getLoad(17);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('get_load');
    });

    test('Robot.getTemperature creates get_temperature block', () => {
      const result = jsToBlockTree('let temp = Robot.getTemperature(17);');
      expect(result.success).toBe(true);
      expect(result.blocks[0].values.VALUE.type).toBe('get_temperature');
    });
  });

  describe('Multiple statements', () => {
    test('multiple statements produce multiple blocks', () => {
      const result = jsToBlockTree(`
        Robot.setPositionLimited(17, 2048);
        wait(1);
        logConsole("done");
      `);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[0].type).toBe('set_joint');
      expect(result.blocks[1].type).toBe('wait');
      expect(result.blocks[2].type).toBe('log');
    });
  });

  describe('Complex examples', () => {
    test('log all positions example', () => {
      const code = `
        var positions = Robot.getAllPositions();
        for (var i = 0; i < positions.length; i++) {
          logConsole("Motor " + (11 + i) + " position: " + positions[i]);
        }
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(2);

      // First block: variable assignment
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].values.VALUE.type).toBe('get_all_joints');

      // Second block: for loop
      expect(result.blocks[1].type).toBe('controls_for');
      expect(result.blocks[1].values.TO.type).toBe('lists_length');
    });

    test('getAllBlockTypes helper works', () => {
      const result = jsToBlockTree('Robot.setPositionLimited(17, 2048);');
      const types = getAllBlockTypes(result.blocks[0]);
      expect(types).toContain('set_joint');
      expect(types).toContain('math_number');
    });

    test('temperature monitoring program', () => {
      const code = `
        Robot.setTorque(17, true);
        var temp = Robot.getTemperature(17);
        if (temp > 50) {
          logConsole("Motor 17 is hot: " + temp);
          Robot.setTorque(17, false);
        }
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[0].type).toBe('enable_torque');
      expect(result.blocks[1].type).toBe('variables_set');
      expect(result.blocks[1].values.VALUE.type).toBe('get_temperature');
      expect(result.blocks[2].type).toBe('controls_if');
    });

    test('load monitoring with while loop', () => {
      const code = `
        Robot.setTorque(17, true);
        Robot.setPositionLimited(17, 3000);
        while (Robot.isMoving(17)) {
          var load = Robot.getLoad(17);
          logConsole("Load: " + load);
          wait(0.1);
        }
        logConsole("Movement complete");
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(4); // 4 top-level blocks
      expect(result.blocks[0].type).toBe('enable_torque');
      expect(result.blocks[1].type).toBe('set_joint');
      expect(result.blocks[2].type).toBe('controls_whileUntil');
      expect(result.blocks[2].values.BOOL.type).toBe('is_moving');
      // Check while loop body has 3 statements
      expect(result.blocks[2].statements.DO.length).toBe(3);
      expect(result.blocks[2].statements.DO[0].type).toBe('variables_set');
      expect(result.blocks[2].statements.DO[0].values.VALUE.type).toBe('get_load');
      expect(result.blocks[2].statements.DO[1].type).toBe('log');
      expect(result.blocks[2].statements.DO[2].type).toBe('wait');
      expect(result.blocks[3].type).toBe('log');
    });

    test('torque enable/disable sequence', () => {
      const code = `
        Robot.setTorqueMultiple(Robot.motorIds, true);
        wait(1);
        Robot.setTorqueMultiple(Robot.motorIds, false);
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[0].type).toBe('enable_all');
      expect(result.blocks[1].type).toBe('wait');
      expect(result.blocks[2].type).toBe('disable_all');
    });

    test('motor position read and modify', () => {
      const code = `
        var pos = Robot.getPosition(17);
        pos = pos + 100;
        Robot.setPositionLimited(17, pos);
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].values.VALUE.type).toBe('get_joint');
      expect(result.blocks[1].type).toBe('variables_set');
      expect(result.blocks[1].values.VALUE.type).toBe('math_arithmetic');
      expect(result.blocks[2].type).toBe('set_joint');
    });

    test('degrees conversion workflow', () => {
      const code = `
        var deg = Robot.getDegrees(17);
        deg += 10;
        Robot.setDegrees(17, deg);
        logJoint(17);
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(4);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[0].values.VALUE.type).toBe('get_degrees');
      expect(result.blocks[1].type).toBe('variables_set'); // deg += 10
      expect(result.blocks[1].values.VALUE.type).toBe('math_arithmetic');
      expect(result.blocks[2].type).toBe('set_degrees');
      expect(result.blocks[3].type).toBe('log_joint');
    });

    test('smooth movement with speed control', () => {
      const code = `
        Robot.setTorque(17, true);
        Robot.setSpeed(17, 100);
        Robot.moveSmooth(17, 3000, 2000);
        wait(2);
        Robot.moveSmooth(17, 1000, 2000);
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(5);
      expect(result.blocks[0].type).toBe('enable_torque');
      expect(result.blocks[1].type).toBe('set_speed');
      expect(result.blocks[2].type).toBe('move_smooth');
      expect(result.blocks[3].type).toBe('wait');
      expect(result.blocks[4].type).toBe('move_smooth');
    });

    test('kinematics coordinate transformation', () => {
      const code = `
        var positions = Robot.getAllPositions();
        var degrees = Robot.jointsToDegrees(positions);
        var coords = Robot.jointsToCoordinates(degrees);
        logConsole("X: " + coords[0]);
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(4);
      expect(result.blocks[0].values.VALUE.type).toBe('get_all_joints');
      expect(result.blocks[1].values.VALUE.type).toBe('joints_to_degrees');
      expect(result.blocks[2].values.VALUE.type).toBe('joints_to_coordinates');
      expect(result.blocks[3].type).toBe('log');
    });

    test('array modification in loop', () => {
      const code = `
        var positions = Robot.getAllPositions();
        for (var i = 0; i < 8; i++) {
          positions[i] = positions[i] + 100;
        }
        Robot.setAllPositions(positions);
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[0].type).toBe('variables_set');
      expect(result.blocks[1].type).toBe('controls_for');
      // Check the for loop body has array assignment
      expect(result.blocks[1].statements.DO[0].type).toBe('lists_setIndex');
      expect(result.blocks[2].type).toBe('set_all_joints');
    });

    test('ping and reboot sequence', () => {
      const code = `
        Robot.ping(17);
        Robot.reboot(17);
        wait(1);
        Robot.ping(17);
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(4);
      expect(result.blocks[0].type).toBe('ping_joint');
      expect(result.blocks[1].type).toBe('reboot_joint');
      expect(result.blocks[2].type).toBe('wait');
      expect(result.blocks[3].type).toBe('ping_joint');
    });

    test('copy joint and check motors', () => {
      const code = `
        Robot.checkAllMotors();
        Robot.copyJoint(17, 18);
        logConsole("Copied joint 17 to 18");
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[0].type).toBe('check_joints');
      expect(result.blocks[1].type).toBe('copy_joint');
      expect(result.blocks[2].type).toBe('log');
    });

    test('current coordinates workflow', () => {
      const code = `
        var coords = Robot.getCurrentCoordinates();
        coords[2] += 5;
        var newJoints = Robot.coordinatesToJoints(coords);
        var newPositions = Robot.degreesToJoints(newJoints);
        Robot.setAllPositions(newPositions);
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(5);
      expect(result.blocks[0].values.VALUE.type).toBe('get_current_coordinates');
      expect(result.blocks[1].type).toBe('lists_setIndex'); // coords[2] += 5
      expect(result.blocks[2].values.VALUE.type).toBe('coordinates_to_joints');
      expect(result.blocks[3].values.VALUE.type).toBe('degrees_to_joints');
      expect(result.blocks[4].type).toBe('set_all_joints');
    });

    test('conditional motor control with AND/OR', () => {
      const code = `
        var temp = Robot.getTemperature(17);
        var load = Robot.getLoad(17);
        if (temp > 50 || load > 100) {
          Robot.setTorque(17, false);
          alert("Motor 17 disabled for safety");
        }
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[2].type).toBe('controls_if');
      expect(result.blocks[2].values.IF0.type).toBe('logic_operation');
      expect(result.blocks[2].values.IF0.fields.OP).toBe('OR');
    });

    test('movement loop with increment', () => {
      const code = `
        Robot.setTorque(17, true);
        var deg = Robot.getDegrees(17);
        for (var i = 0; i < 10; i++) {
          deg = deg + 1;
          Robot.setDegrees(17, deg);
          logJoint(17);
          wait(1);
        }
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[0].type).toBe('enable_torque');
      expect(result.blocks[1].type).toBe('variables_set');
      expect(result.blocks[2].type).toBe('controls_for');
      // Check loop body has 4 statements
      expect(result.blocks[2].statements.DO.length).toBe(4);
      expect(result.blocks[2].statements.DO[0].type).toBe('variables_set'); // deg = deg + 1
      expect(result.blocks[2].statements.DO[1].type).toBe('set_degrees');
      expect(result.blocks[2].statements.DO[2].type).toBe('log_joint');
      expect(result.blocks[2].statements.DO[3].type).toBe('wait');
    });

    test('head nodding with coordinate modification and if-else', () => {
      const code = `
        Robot.setTorqueMultiple(Robot.motorIds, true);
        for (var i = 0; i < 20; i++) {
          var coords = Robot.getCurrentCoordinates();
          if (i % 2 == 0) {
            coords[2] = coords[2] + 1;
          } else {
            coords[2] = coords[2] - 1;
          }
          var newAngles = Robot.coordinatesToJoints(coords);
          Robot.setAllPositions(Robot.degreesToJoints(newAngles));
          wait(1);
        }
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);
      expect(result.blocks.length).toBe(2);
      expect(result.blocks[0].type).toBe('enable_all');
      expect(result.blocks[1].type).toBe('controls_for');
      // Check for loop body
      const loopBody = result.blocks[1].statements.DO;
      expect(loopBody.length).toBe(5);
      expect(loopBody[0].type).toBe('variables_set'); // var coords = ...
      expect(loopBody[0].values.VALUE.type).toBe('get_current_coordinates');
      expect(loopBody[1].type).toBe('controls_if'); // if-else
      expect(loopBody[1].extra.hasElse).toBe(true);
      expect(loopBody[2].type).toBe('variables_set'); // var newAngles = ...
      expect(loopBody[2].values.VALUE.type).toBe('coordinates_to_joints');
      expect(loopBody[3].type).toBe('set_all_joints');
      expect(loopBody[4].type).toBe('wait');
    });
  });

  describe('Error handling', () => {
    test('invalid JS returns error', () => {
      const result = jsToBlockTree('this is not valid js {{{');
      expect(result.success).toBe(false);
      expect(result.error).toBeDefined();
    });

    test('unknown function calls return null/undefined', () => {
      const result = jsToBlockTree('unknownFunction();');
      expect(result.success).toBe(true);
      expect(result.blocks[0]).toBeFalsy();
    });
  });

  // Block structure validation helpers
  function validateBlockConnections(block, path = 'root') {
    const errors = [];

    if (!block) return errors;

    // Check that the block has a type
    if (!block.type) {
      errors.push(`${path}: block missing type`);
      return errors;
    }

    // Check values are properly connected (not null/undefined where expected)
    if (block.values) {
      for (const [key, value] of Object.entries(block.values)) {
        if (value === null || value === undefined) {
          errors.push(`${path}.values.${key}: value is null/undefined`);
        } else if (typeof value === 'object') {
          errors.push(...validateBlockConnections(value, `${path}.values.${key}`));
        }
      }
    }

    // Check statements are properly formed
    if (block.statements) {
      for (const [key, statements] of Object.entries(block.statements)) {
        if (!Array.isArray(statements)) {
          errors.push(`${path}.statements.${key}: should be array, got ${typeof statements}`);
        } else {
          statements.forEach((stmt, i) => {
            if (stmt === null || stmt === undefined) {
              errors.push(`${path}.statements.${key}[${i}]: statement is null/undefined`);
            } else {
              errors.push(...validateBlockConnections(stmt, `${path}.statements.${key}[${i}]`));
            }
          });
        }
      }
    }

    return errors;
  }

  function validateBlockTree(result) {
    if (!result.success) return { valid: false, errors: ['Parse failed: ' + result.error] };

    const errors = [];
    result.blocks.forEach((block, i) => {
      if (block) {
        errors.push(...validateBlockConnections(block, `block[${i}]`));
      }
    });

    return { valid: errors.length === 0, errors };
  }

  describe('Block structure validation', () => {
    test('Simple assignment has complete structure', () => {
      const result = jsToBlockTree('var x = 10;');
      const validation = validateBlockTree(result);
      expect(validation.valid).toBe(true);
      expect(validation.errors).toHaveLength(0);
    });

    test('Nested function call has complete structure', () => {
      const result = jsToBlockTree('Robot.setAllPositions(Robot.degreesToJoints(joints));');
      const validation = validateBlockTree(result);
      expect(validation.valid).toBe(true);
      expect(validation.errors).toHaveLength(0);
    });

    test('Array with expressions has complete structure', () => {
      const result = jsToBlockTree(`
        var currentCoords = Robot.getCurrentCoordinates();
        var coordsUp = [currentCoords[0], currentCoords[1], currentCoords[2] + 10, currentCoords[3], currentCoords[4], currentCoords[5]];
      `);
      const validation = validateBlockTree(result);
      expect(validation.valid).toBe(true);
      expect(validation.errors).toHaveLength(0);

      // Verify the array structure specifically
      const arrayBlock = result.blocks[1].values.VALUE;
      expect(arrayBlock.type).toBe('lists_create_with');
      expect(arrayBlock.extra.itemCount).toBe(6);
      // All 6 items should have values
      for (let i = 0; i < 6; i++) {
        expect(arrayBlock.values[`ADD${i}`]).toBeDefined();
        expect(arrayBlock.values[`ADD${i}`].type).toBeDefined();
      }
    });

    test('Kinematics workflow has complete structure', () => {
      const code = `
        Robot.setTorqueMultiple(Robot.motorIds, true);
        var baseCoords = Robot.getCurrentCoordinates();
        for (var i = 0; i < 5; i++) {
          var upCoords = [baseCoords[0], baseCoords[1], baseCoords[2] + 5, baseCoords[3], baseCoords[4], baseCoords[5]];
          var upJoints = Robot.coordinatesToJoints(upCoords);
          Robot.setAllPositions(Robot.degreesToJoints(upJoints));
          wait(0.5);
        }
      `;
      const result = jsToBlockTree(code);
      const validation = validateBlockTree(result);
      expect(validation.valid).toBe(true);
      expect(validation.errors).toHaveLength(0);
    });

    test('For loop with if-else has complete structure', () => {
      const code = `
        for (var i = 0; i < 10; i++) {
          if (i % 2 == 0) {
            logConsole("even");
          } else {
            logConsole("odd");
          }
        }
      `;
      const result = jsToBlockTree(code);
      const validation = validateBlockTree(result);
      expect(validation.valid).toBe(true);
      expect(validation.errors).toHaveLength(0);

      // Verify loop structure
      const loop = result.blocks[0];
      expect(loop.type).toBe('controls_for');
      expect(loop.statements.DO).toBeDefined();
      expect(loop.statements.DO.length).toBe(1);

      // Verify if-else structure
      const ifBlock = loop.statements.DO[0];
      expect(ifBlock.type).toBe('controls_if');
      expect(ifBlock.statements.DO0).toBeDefined();
      expect(ifBlock.statements.ELSE).toBeDefined();
    });

    test('Complex head nodding example', () => {
      const code = `
        Robot.setTorqueMultiple(Robot.motorIds, true);
        var currentCoords = Robot.getCurrentCoordinates();
        for (var i = 0; i < 10; i++) {
          var coordsUp = [currentCoords[0], currentCoords[1], currentCoords[2] + 10, currentCoords[3], currentCoords[4], currentCoords[5]];
          var jointsUp = Robot.coordinatesToJoints(coordsUp);
          Robot.setAllPositions(Robot.degreesToJoints(jointsUp));
          wait(1);
          var coordsDown = [currentCoords[0], currentCoords[1], currentCoords[2] - 10, currentCoords[3], currentCoords[4], currentCoords[5]];
          var jointsDown = Robot.coordinatesToJoints(coordsDown);
          Robot.setAllPositions(Robot.degreesToJoints(jointsDown));
          wait(1);
        }
      `;
      const result = jsToBlockTree(code);
      expect(result.success).toBe(true);

      const validation = validateBlockTree(result);
      expect(validation.valid).toBe(true);
      if (validation.errors.length > 0) {
        console.log('Validation errors:', validation.errors);
      }
      expect(validation.errors).toHaveLength(0);

      // Verify structure
      expect(result.blocks.length).toBe(3);
      expect(result.blocks[0].type).toBe('enable_all');
      expect(result.blocks[1].type).toBe('variables_set');
      expect(result.blocks[1].values.VALUE.type).toBe('get_current_coordinates');
      expect(result.blocks[2].type).toBe('controls_for');

      // Loop should have 8 statements (4 per iteration pattern x2)
      const loopBody = result.blocks[2].statements.DO;
      expect(loopBody.length).toBe(8);

      // Check array blocks inside loop
      const arrayBlock1 = loopBody[0].values.VALUE;
      expect(arrayBlock1.type).toBe('lists_create_with');
      expect(arrayBlock1.extra.itemCount).toBe(6);

      // Verify array element 2 has arithmetic
      expect(arrayBlock1.values.ADD2.type).toBe('math_arithmetic');
      expect(arrayBlock1.values.ADD2.fields.OP).toBe('ADD');
    });

    test('String concatenation in loop', () => {
      const code = `
        var positions = Robot.getAllPositions();
        for (var i = 0; i < 8; i++) {
          logConsole("Motor " + (11 + i) + " position: " + positions[i]);
        }
      `;
      const result = jsToBlockTree(code);
      const validation = validateBlockTree(result);
      expect(validation.valid).toBe(true);
      expect(validation.errors).toHaveLength(0);
    });

    test('While loop with conditions', () => {
      const code = `
        var x = 0;
        while (x < 100) {
          x = x + 10;
          logConsole("x is now: " + x);
        }
      `;
      const result = jsToBlockTree(code);
      const validation = validateBlockTree(result);
      expect(validation.valid).toBe(true);
      expect(validation.errors).toHaveLength(0);
    });
  });
});
