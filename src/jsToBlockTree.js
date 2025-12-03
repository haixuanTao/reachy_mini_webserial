/**
 * JS to Block Tree Converter
 * Parses JavaScript code and returns a tree structure representing Blockly blocks.
 * This module can be tested without Blockly.
 */

// Use acorn for parsing (will be provided externally in browser, required in Node)
var acorn;
if (typeof require !== 'undefined') {
  acorn = require('acorn');
}

function jsToBlockTree(jsCode, acornLib) {
  var parser = acornLib || acorn;
  if (!parser) {
    throw new Error('Acorn parser not available');
  }

  try {
    var ast = parser.parse(jsCode, { ecmaVersion: 2020, sourceType: 'script' });
    var statements = [];

    for (var i = 0; i < ast.body.length; i++) {
      var blockTree = processStatement(ast.body[i]);
      if (blockTree) {
        statements.push(blockTree);
      }
    }

    return { success: true, blocks: statements };
  } catch (e) {
    return { success: false, error: e.message };
  }
}

function processStatement(node) {
  if (!node) return null;

  // Expression statement (function calls)
  if (node.type === 'ExpressionStatement') {
    return processExpression(node.expression, true);
  }

  // Variable declaration: let x = value;
  if (node.type === 'VariableDeclaration') {
    var declarations = node.declarations;
    var blocks = [];

    for (var i = 0; i < declarations.length; i++) {
      var decl = declarations[i];
      var varName = decl.id.name;
      var valueTree = decl.init ? processExpression(decl.init, false) : null;

      blocks.push({
        type: 'variables_set',
        fields: { VAR: varName },
        values: valueTree ? { VALUE: valueTree } : {}
      });
    }

    // Return chained if multiple, otherwise single
    if (blocks.length === 1) return blocks[0];
    return { type: '_chain', blocks: blocks };
  }

  // For loop: for (let i = start; i < end; i++)
  if (node.type === 'ForStatement') {
    var varName = 'i';
    var fromTree = null;
    var toTree = null;
    var bodyBlocks = [];

    if (node.init && node.init.declarations) {
      varName = node.init.declarations[0].id.name;
      if (node.init.declarations[0].init) {
        fromTree = processExpression(node.init.declarations[0].init, false);
      }
    }

    if (node.test && node.test.right) {
      toTree = processExpression(node.test.right, false);
    }

    if (node.body && node.body.body) {
      for (var i = 0; i < node.body.body.length; i++) {
        var bodyBlock = processStatement(node.body.body[i]);
        if (bodyBlock) bodyBlocks.push(bodyBlock);
      }
    }

    return {
      type: 'controls_for',
      fields: { VAR: varName },
      values: {
        FROM: fromTree || { type: 'math_number', fields: { NUM: 0 } },
        TO: toTree,
        BY: { type: 'math_number', fields: { NUM: 1 } }
      },
      statements: { DO: bodyBlocks }
    };
  }

  // While loop
  if (node.type === 'WhileStatement') {
    var condTree = processExpression(node.test, false);
    var bodyBlocks = [];

    if (node.body && node.body.body) {
      for (var i = 0; i < node.body.body.length; i++) {
        var bodyBlock = processStatement(node.body.body[i]);
        if (bodyBlock) bodyBlocks.push(bodyBlock);
      }
    }

    return {
      type: 'controls_whileUntil',
      values: { BOOL: condTree },
      statements: { DO: bodyBlocks }
    };
  }

  // If statement
  if (node.type === 'IfStatement') {
    var condTree = processExpression(node.test, false);
    var bodyBlocks = [];
    var elseBlocks = [];

    if (node.consequent && node.consequent.body) {
      for (var i = 0; i < node.consequent.body.length; i++) {
        var bodyBlock = processStatement(node.consequent.body[i]);
        if (bodyBlock) bodyBlocks.push(bodyBlock);
      }
    }

    // Handle else clause
    if (node.alternate) {
      if (node.alternate.body) {
        for (var i = 0; i < node.alternate.body.length; i++) {
          var elseBlock = processStatement(node.alternate.body[i]);
          if (elseBlock) elseBlocks.push(elseBlock);
        }
      }
    }

    var result = {
      type: 'controls_if',
      values: { IF0: condTree },
      statements: { DO0: bodyBlocks }
    };

    if (elseBlocks.length > 0) {
      result.extra = { hasElse: true };
      result.statements.ELSE = elseBlocks;
    }

    return result;
  }

  return null;
}

function processExpression(node, asStatement) {
  if (!node) return null;

  // Await expression
  if (node.type === 'AwaitExpression') {
    return processExpression(node.argument, asStatement);
  }

  // Assignment expression: x = value, x += value, etc.
  if (node.type === 'AssignmentExpression') {
    // Handle array index assignment: arr[i] = value
    if (node.left.type === 'MemberExpression' && node.left.computed) {
      var listTree = processExpression(node.left.object, false);
      var indexTree = processExpression(node.left.property, false);
      var valueTree = processExpression(node.right, false);
      return {
        type: 'lists_setIndex',
        fields: { MODE: 'SET', WHERE: 'FROM_START' },
        values: {
          LIST: listTree,
          AT: indexTree,
          TO: valueTree
        }
      };
    }

    var varName = node.left.name;

    // Handle compound assignment: +=, -=, *=, /=
    if (node.operator !== '=') {
      var opMap = { '+=': 'ADD', '-=': 'MINUS', '*=': 'MULTIPLY', '/=': 'DIVIDE', '%=': 'MODULO' };
      var op = opMap[node.operator];
      if (op) {
        return {
          type: 'variables_set',
          fields: { VAR: varName },
          values: {
            VALUE: {
              type: 'math_arithmetic',
              fields: { OP: op },
              values: {
                A: { type: 'variables_get', fields: { VAR: varName } },
                B: processExpression(node.right, false)
              }
            }
          }
        };
      }
    }

    // Simple assignment: x = value
    var valueTree = processExpression(node.right, false);
    return {
      type: 'variables_set',
      fields: { VAR: varName },
      values: valueTree ? { VALUE: valueTree } : {}
    };
  }

  // Call expression
  if (node.type === 'CallExpression') {
    // Handle Math functions
    if (node.callee.type === 'MemberExpression' &&
        node.callee.object.name === 'Math') {
      var mathFunc = node.callee.property.name;
      var mathOpMap = {
        'floor': 'ROUNDDOWN',
        'ceil': 'ROUNDUP',
        'round': 'ROUND',
        'abs': 'ABS',
        'sqrt': 'ROOT',
        'sin': 'SIN',
        'cos': 'COS',
        'tan': 'TAN'
      };
      if (mathOpMap[mathFunc] && node.arguments.length >= 1) {
        return {
          type: 'math_single',
          fields: { OP: mathOpMap[mathFunc] },
          values: { NUM: processExpression(node.arguments[0], false) }
        };
      }
    }
    return processCall(node, asStatement);
  }

  // Number literal
  if (node.type === 'Literal' && typeof node.value === 'number') {
    return { type: 'math_number', fields: { NUM: node.value } };
  }

  // String literal
  if (node.type === 'Literal' && typeof node.value === 'string') {
    return { type: 'text', fields: { TEXT: node.value } };
  }

  // Boolean literal
  if (node.type === 'Literal' && typeof node.value === 'boolean') {
    return { type: 'logic_boolean', fields: { BOOL: node.value ? 'TRUE' : 'FALSE' } };
  }

  // Unary expression: -x, !x
  if (node.type === 'UnaryExpression') {
    if (node.operator === '-') {
      return {
        type: 'math_single',
        fields: { OP: 'NEG' },
        values: { NUM: processExpression(node.argument, false) }
      };
    }
    if (node.operator === '!') {
      return {
        type: 'logic_negate',
        values: { BOOL: processExpression(node.argument, false) }
      };
    }
  }

  // Logical expression: && ||
  if (node.type === 'LogicalExpression') {
    return {
      type: 'logic_operation',
      fields: { OP: node.operator === '&&' ? 'AND' : 'OR' },
      values: {
        A: processExpression(node.left, false),
        B: processExpression(node.right, false)
      }
    };
  }

  // Binary expression (comparisons, math)
  if (node.type === 'BinaryExpression') {
    var opMap = {
      '==': 'EQ', '===': 'EQ', '!=': 'NEQ', '!==': 'NEQ',
      '<': 'LT', '<=': 'LTE', '>': 'GT', '>=': 'GTE',
      '+': 'ADD', '-': 'MINUS', '*': 'MULTIPLY', '/': 'DIVIDE', '%': 'MODULO'
    };

    if (['==', '===', '!=', '!==', '<', '<=', '>', '>='].includes(node.operator)) {
      return {
        type: 'logic_compare',
        fields: { OP: opMap[node.operator] },
        values: {
          A: processExpression(node.left, false),
          B: processExpression(node.right, false)
        }
      };
    }

    // Check if this is string concatenation
    if (node.operator === '+') {
      var parts = flattenStringConcat(node);
      var hasString = parts.some(function(p) {
        return p.type === 'Literal' && typeof p.value === 'string';
      });

      if (hasString) {
        var items = parts.map(function(p) {
          return processExpression(p, false);
        });
        var values = {};
        for (var i = 0; i < items.length; i++) {
          values['ADD' + i] = items[i];
        }
        return {
          type: 'text_join',
          extra: { itemCount: items.length },
          values: values
        };
      }
    }

    if (['+', '-', '*', '/', '%'].includes(node.operator)) {
      return {
        type: 'math_arithmetic',
        fields: { OP: opMap[node.operator] },
        values: {
          A: processExpression(node.left, false),
          B: processExpression(node.right, false)
        }
      };
    }
  }

  // Identifier (variable)
  if (node.type === 'Identifier') {
    return { type: 'variables_get', fields: { VAR: node.name } };
  }

  // Member expression: obj.prop or arr[i]
  if (node.type === 'MemberExpression') {
    // arr.length -> lists_length
    if (!node.computed && node.property.name === 'length') {
      return {
        type: 'lists_length',
        values: { VALUE: processExpression(node.object, false) }
      };
    }

    // arr[i] -> lists_getIndex
    if (node.computed) {
      return {
        type: 'lists_getIndex',
        fields: { MODE: 'GET', WHERE: 'FROM_START' },
        values: {
          VALUE: processExpression(node.object, false),
          AT: processExpression(node.property, false)
        }
      };
    }
  }

  // Array expression [1, 2, 3]
  if (node.type === 'ArrayExpression') {
    var values = {};
    for (var i = 0; i < node.elements.length; i++) {
      values['ADD' + i] = processExpression(node.elements[i], false);
    }
    return {
      type: 'lists_create_with',
      extra: { itemCount: node.elements.length },
      values: values
    };
  }

  return null;
}

function processCall(node, asStatement) {
  var callee = getCalleeName(node.callee);
  var args = node.arguments;

  // Robot.setPositionLimited(motor, pos) -> set_joint
  if (callee === 'Robot.setPositionLimited' && args.length >= 2) {
    return {
      type: 'set_joint',
      fields: { MOTOR: getArgValue(args[0]) },
      values: { JOINT: processExpression(args[1], false) }
    };
  }

  // Robot.getPosition(motor) -> get_joint
  if (callee === 'Robot.getPosition' && args.length >= 1) {
    return {
      type: 'get_joint',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // Robot.setTorque(motor, true/false) -> enable_torque/disable_torque
  if (callee === 'Robot.setTorque' && args.length >= 2) {
    var enable = args[1].value === true;
    return {
      type: enable ? 'enable_torque' : 'disable_torque',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // Robot.setTorqueMultiple -> enable_all / disable_all
  if (callee === 'Robot.setTorqueMultiple' && args.length >= 2) {
    var enable = args[1].value === true;
    return { type: enable ? 'enable_all' : 'disable_all' };
  }

  // logConsole(msg) -> log
  if (callee === 'logConsole') {
    return {
      type: 'log',
      values: args.length >= 1 ? { MESSAGE: processExpression(args[0], false) } : {}
    };
  }

  // alert(msg) -> alert
  if (callee === 'alert') {
    return {
      type: 'alert',
      values: args.length >= 1 ? { MESSAGE: processExpression(args[0], false) } : {}
    };
  }

  // sleep(ms) -> wait_ms
  if (callee === 'sleep') {
    return {
      type: 'wait_ms',
      values: args.length >= 1 ? { TIME: processExpression(args[0], false) } : {}
    };
  }

  // wait(seconds) -> wait
  if (callee === 'wait') {
    return {
      type: 'wait',
      values: args.length >= 1 ? { TIME: processExpression(args[0], false) } : {}
    };
  }

  // Robot.moveSmooth(motor, pos, duration) -> move_smooth
  if (callee === 'Robot.moveSmooth' && args.length >= 3) {
    return {
      type: 'move_smooth',
      fields: { MOTOR: getArgValue(args[0]) },
      values: {
        JOINT: processExpression(args[1], false),
        DURATION: processExpression(args[2], false)
      }
    };
  }

  // Robot.checkAllMotors() -> check_joints
  if (callee === 'Robot.checkAllMotors') {
    return { type: 'check_joints' };
  }

  // Robot.setDegrees(motor, degrees) -> set_degrees
  if (callee === 'Robot.setDegrees' && args.length >= 2) {
    return {
      type: 'set_degrees',
      fields: { MOTOR: getArgValue(args[0]) },
      values: { DEGREES: processExpression(args[1], false) }
    };
  }

  // Robot.getDegrees(motor) -> get_degrees
  if (callee === 'Robot.getDegrees' && args.length >= 1) {
    return {
      type: 'get_degrees',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // Robot.getAllPositions() -> get_all_joints
  if (callee === 'Robot.getAllPositions') {
    return { type: 'get_all_joints' };
  }

  // Robot.setAllPositions(arr) -> set_all_joints
  if (callee === 'Robot.setAllPositions' && args.length >= 1) {
    return {
      type: 'set_all_joints',
      values: { JOINTS: processExpression(args[0], false) }
    };
  }

  // Robot.copyJoint(from, to) -> copy_joint
  if (callee === 'Robot.copyJoint' && args.length >= 2) {
    return {
      type: 'copy_joint',
      fields: { FROM: getArgValue(args[0]), TO: getArgValue(args[1]) }
    };
  }

  // Robot.jointsToCoordinates(arr) -> joints_to_coordinates
  if (callee === 'Robot.jointsToCoordinates' && args.length >= 1) {
    return {
      type: 'joints_to_coordinates',
      values: { JOINTS: processExpression(args[0], false) }
    };
  }

  // Robot.coordinatesToJoints(arr) -> coordinates_to_joints
  if (callee === 'Robot.coordinatesToJoints' && args.length >= 1) {
    return {
      type: 'coordinates_to_joints',
      values: { COORDINATES: processExpression(args[0], false) }
    };
  }

  // Robot.getCurrentCoordinates() -> get_current_coordinates
  if (callee === 'Robot.getCurrentCoordinates') {
    return { type: 'get_current_coordinates' };
  }

  // Robot.jointsToDegrees(arr) -> joints_to_degrees
  if (callee === 'Robot.jointsToDegrees' && args.length >= 1) {
    return {
      type: 'joints_to_degrees',
      values: { JOINTS: processExpression(args[0], false) }
    };
  }

  // Robot.degreesToJoints(arr) -> degrees_to_joints
  if (callee === 'Robot.degreesToJoints' && args.length >= 1) {
    return {
      type: 'degrees_to_joints',
      values: { DEGREES: processExpression(args[0], false) }
    };
  }

  // Robot.isMoving(motor) -> is_moving
  if (callee === 'Robot.isMoving' && args.length >= 1) {
    return {
      type: 'is_moving',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // Robot.getLoad(motor) -> get_load
  if (callee === 'Robot.getLoad' && args.length >= 1) {
    return {
      type: 'get_load',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // Robot.getTemperature(motor) -> get_temperature
  if (callee === 'Robot.getTemperature' && args.length >= 1) {
    return {
      type: 'get_temperature',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // logJoint(motor) -> log_joint
  if (callee === 'logJoint' && args.length >= 1) {
    return {
      type: 'log_joint',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // Robot.ping(motor) -> ping_joint
  if (callee === 'Robot.ping' && args.length >= 1) {
    return {
      type: 'ping_joint',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // Robot.reboot(motor) -> reboot_joint
  if (callee === 'Robot.reboot' && args.length >= 1) {
    return {
      type: 'reboot_joint',
      fields: { MOTOR: getArgValue(args[0]) }
    };
  }

  // Robot.rebootAll() -> reboot_all
  if (callee === 'Robot.rebootAll') {
    return { type: 'reboot_all' };
  }

  // Robot.setSpeed(motor, speed) -> set_speed
  if (callee === 'Robot.setSpeed' && args.length >= 2) {
    return {
      type: 'set_speed',
      fields: { MOTOR: getArgValue(args[0]) },
      values: { SPEED: processExpression(args[1], false) }
    };
  }

  return null;
}

function getCalleeName(callee) {
  if (callee.type === 'Identifier') {
    return callee.name;
  }
  if (callee.type === 'MemberExpression') {
    var obj = callee.object.name || '';
    var prop = callee.property.name || '';
    return obj + '.' + prop;
  }
  return '';
}

function getArgValue(arg) {
  if (arg.type === 'Literal') {
    return arg.value;
  }
  return null;
}

function flattenStringConcat(node) {
  var parts = [];
  function collect(n) {
    if (n.type === 'BinaryExpression' && n.operator === '+') {
      collect(n.left);
      collect(n.right);
    } else {
      parts.push(n);
    }
  }
  collect(node);
  return parts;
}

// Helper to get all block types from a block tree
function getAllBlockTypes(blockTree) {
  var types = [];

  function collect(node) {
    if (!node) return;
    if (node.type && node.type !== '_chain') {
      types.push(node.type);
    }
    if (node.blocks) {
      node.blocks.forEach(collect);
    }
    if (node.values) {
      Object.values(node.values).forEach(collect);
    }
    if (node.statements) {
      Object.values(node.statements).forEach(function(stmts) {
        if (Array.isArray(stmts)) {
          stmts.forEach(collect);
        }
      });
    }
  }

  collect(blockTree);
  return types;
}

// Export for Node.js
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    jsToBlockTree: jsToBlockTree,
    getAllBlockTypes: getAllBlockTypes
  };
}
