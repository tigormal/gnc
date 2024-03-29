
# parsetab.py
# This file is automatically generated. Do not edit.
# pylint: disable=W,C,R
_tabversion = '3.10'

_lr_method = 'LALR'

_lr_signature = 'ABORT APPROACH AREA COMMA COMMAND DIR DISCOVERY DISTANCE EQUAL EQUALS EXIT FOLLOW GREATER GREATEREQUAL ID INT LESSEQUAL LESSTHAN LPAREN MINUS NOTEQUAL PLUS REAL REPEAT RPAREN SPEED STRING TARGET TELEMETRY TIMES UNTIL VISUAL\n    stmt : TARGET STRING COMMA args\n         | TARGET STRING\n         | TARGET STRING shift\n         | TARGET STRING shift COMMA args\n    \n    stmt : FOLLOW STRING COMMA args\n         | FOLLOW STRING\n    \n    shift : DIR\n          | DIR DIR\n          | DIR DIR DIR\n    \n    args : args COMMA expr\n         | expr\n    \n    expr : DISTANCE INT\n         | DISTANCE REAL\n    \n    expr : AREA STRING\n    \n    expr : VISUAL ID\n    \n    stmt : COMMAND ID\n    \n    stmt : ABORT\n    \n    expr : APPROACH\n    \n    expr : SPEED INT\n         | SPEED REAL\n    \n    stmt : DISCOVERY COMMA expr\n    '
    
_lr_action_items = {'TARGET':([0,],[2,]),'FOLLOW':([0,],[3,]),'COMMAND':([0,],[4,]),'ABORT':([0,],[5,]),'DISCOVERY':([0,],[6,]),'$end':([1,5,7,8,9,12,13,15,19,21,22,24,25,26,27,28,29,30,31,33,34,35,],[0,-17,-2,-6,-16,-3,-7,-21,-18,-1,-11,-8,-5,-12,-13,-14,-15,-19,-20,-4,-9,-10,]),'STRING':([2,3,17,],[7,8,28,]),'ID':([4,18,],[9,29,]),'COMMA':([6,7,8,12,13,19,21,22,24,25,26,27,28,29,30,31,33,34,35,],[10,11,14,23,-7,-18,32,-11,-8,32,-12,-13,-14,-15,-19,-20,32,-9,-10,]),'DIR':([7,13,24,],[13,24,34,]),'DISTANCE':([10,11,14,23,32,],[16,16,16,16,16,]),'AREA':([10,11,14,23,32,],[17,17,17,17,17,]),'VISUAL':([10,11,14,23,32,],[18,18,18,18,18,]),'APPROACH':([10,11,14,23,32,],[19,19,19,19,19,]),'SPEED':([10,11,14,23,32,],[20,20,20,20,20,]),'INT':([16,20,],[26,30,]),'REAL':([16,20,],[27,31,]),}

_lr_action = {}
for _k, _v in _lr_action_items.items():
   for _x,_y in zip(_v[0],_v[1]):
      if not _x in _lr_action:  _lr_action[_x] = {}
      _lr_action[_x][_k] = _y
del _lr_action_items

_lr_goto_items = {'stmt':([0,],[1,]),'shift':([7,],[12,]),'expr':([10,11,14,23,32,],[15,22,22,22,35,]),'args':([11,14,23,],[21,25,33,]),}

_lr_goto = {}
for _k, _v in _lr_goto_items.items():
   for _x, _y in zip(_v[0], _v[1]):
       if not _x in _lr_goto: _lr_goto[_x] = {}
       _lr_goto[_x][_k] = _y
del _lr_goto_items
_lr_productions = [
  ("S' -> stmt","S'",1,None,None,None),
  ('stmt -> TARGET STRING COMMA args','stmt',4,'p_statement_target','lex.py',83),
  ('stmt -> TARGET STRING','stmt',2,'p_statement_target','lex.py',84),
  ('stmt -> TARGET STRING shift','stmt',3,'p_statement_target','lex.py',85),
  ('stmt -> TARGET STRING shift COMMA args','stmt',5,'p_statement_target','lex.py',86),
  ('stmt -> FOLLOW STRING COMMA args','stmt',4,'p_statement_follow','lex.py',105),
  ('stmt -> FOLLOW STRING','stmt',2,'p_statement_follow','lex.py',106),
  ('shift -> DIR','shift',1,'p_shift','lex.py',121),
  ('shift -> DIR DIR','shift',2,'p_shift','lex.py',122),
  ('shift -> DIR DIR DIR','shift',3,'p_shift','lex.py',123),
  ('args -> args COMMA expr','args',3,'p_args_target','lex.py',135),
  ('args -> expr','args',1,'p_args_target','lex.py',136),
  ('expr -> DISTANCE INT','expr',2,'p_expr_dist','lex.py',146),
  ('expr -> DISTANCE REAL','expr',2,'p_expr_dist','lex.py',147),
  ('expr -> AREA STRING','expr',2,'p_expr_area','lex.py',153),
  ('expr -> VISUAL ID','expr',2,'p_expr_visual','lex.py',159),
  ('stmt -> COMMAND ID','stmt',2,'p_statement_cmd','lex.py',166),
  ('stmt -> ABORT','stmt',1,'p_statement_abort','lex.py',175),
  ('expr -> APPROACH','expr',1,'p_expr_appr','lex.py',183),
  ('expr -> SPEED INT','expr',2,'p_expr_speed','lex.py',189),
  ('expr -> SPEED REAL','expr',2,'p_expr_speed','lex.py',190),
  ('stmt -> DISCOVERY COMMA expr','stmt',3,'p_discovery','lex.py',196),
]
