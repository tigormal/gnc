
import ply.lex as lex # type: ignore
import re

#! Lexer

error_syntax  = 0

keywords = (
    'TARGET', 'COMMAND', 'FOLLOW', 'ABORT', 'DISCOVERY', 'AREA',
    'REPEAT', 'TIMES', 'UNTIL', 'VISUAL', 'EXIT', 'TELEMETRY', 'APPROACH',
    'DISTANCE', 'SPEED'
)

tokens = keywords + (
    'ID', 'GREATER', 'GREATEREQUAL', 'NOTEQUAL', 'EQUAL',
    'LESSEQUAL', 'LESSTHAN', 'MINUS', 'PLUS', 'LPAREN', 'RPAREN', 'STRING', 'INT', 'REAL', 'DIR', 'COMMA', 'EQUALS'
)

rmap = {}
for r in keywords:
    rmap[r] = r

t_COMMA = r','
t_GREATER          = r'>'   #22
t_GREATEREQUAL     = r'>='    #21
t_NOTEQUAL         = r'<>'  #20
t_EQUAL            = r'==' #19
t_LESSEQUAL        = r'<=' #18
t_LESSTHAN         = r'<' #17
t_MINUS            = r'-' #11
t_PLUS             = r'\+' #10
t_LPAREN           = r'\('  #1,2
t_RPAREN           = r'\)' #1,2
t_ignore           = ' \t'

def t_REAL(t):
    r'[-+]?[0-9]+(\.([0-9]+)?([eE][-+]?[0-9]+)?|[eE][-+]?[0-9]+)'
    t.value = float(t.value)
    return t

def t_DIR(t):
    r'[-+]?[X-Z]?[0-9]+(\.([0-9]+)?([eE][-+]?[0-9]+)?|[eE][-+]?[0-9]+)'
    x, y, z = t.value.find('X'), t.value.find('Y'), t.value.find('Z')
    dir = [dir for dir in [x, y, z] if dir != -1][0]
    val = float(t.value[:dir]+t.value[dir+1:])
    dir = t.value[dir]
    t.value = (dir, val) # tuple of direction and value
    return t

def t_INT(t):
    r'[0-9]+'
    # r'[-+]?[0-9]+'
    t.value = int(t.value)
    return t

def t_STRING(t):
     r'(\"([^\"]|(\\.))*\")|((\'([^\"]|(\\.))*\'))'
     # r'(?:\'|\").*(?:\'|\")'
     return t

def t_newline(t):
    r'\n+'
    t.lexer.lineno += len(t.value)

def t_ID(t):
    r'[a-zA-Z][a-zA-Z0-9_]*'
    t.type = rmap.get(t.value, "ID")
    return t

def t_error(t):
    print("SYNTAX ERROR: %s at %d" % (t.value[0], t.lexer.lineno))
    error_syntax = 1
    return
    # t.lexer.skip(1)

lex.lex(debug = 0);

#! Parser

def p_statement_target(p):
    #! TARGET
    '''
    stmt : TARGET STRING COMMA args
         | TARGET STRING
         | TARGET STRING shift
         | TARGET STRING shift COMMA args
    '''
    p[0] = {}
    p[0]['TARGET'] = p[2]
    print(f"len p: {len(p)}")
    print(f"p: {list(p)}")
    if len(p) >= 5:
        print('target parsing args')
        for arg in p[len(p)-1]:
            p[0][arg[0]] = arg[1]
    if len(p) == 4 or len(p) == 6:
        print('target parsing shift')
        p[0]['SHIFT'] = p[3]
    print(p[0])


def p_statement_follow(p):
    #! FOLLOW
    '''
    stmt : FOLLOW STRING COMMA args
         | FOLLOW STRING
    '''
    p[0] = {}
    p[0]['FOLLOW'] = p[2]
    print(f"len p: {len(p)}")
    print(f"p: {list(p)}")
    if len(p) == 5:
        print('follow parsing args')
        for arg in p[len(p)-1]:
            p[0][arg[0]] = arg[1]
    print(p[0])

def p_shift(p):
    '''
    shift : DIR
          | DIR DIR
          | DIR DIR DIR
    '''
    dirs = {'X': 0, 'Y': 1, 'Z': 2}
    p[0] = [0, 0, 0]
    print(f"shift p len: {len(p)}")
    for i in range(1,len(p)):
        arg = p[i]
        p[0][dirs[arg[0]]] = arg[1]
    print(f"shift: {p[0]}")

def p_args_target(p):
    '''
    args : args COMMA expr
         | expr
    '''
    print(f"args: {list(p)}")
    if len(p) == 2:
       p[0] = [p[1]]
    else:
       p[0] = p[1] + [p[3]]

def p_expr_dist(p):
    '''
    expr : DISTANCE INT
         | DISTANCE REAL
    '''
    p[0] = (p[1], p[2])

def p_expr_area(p):
    '''
    expr : AREA STRING
    '''
    p[0] = (p[1], p[2])

def p_expr_visual(p):
    '''
    expr : VISUAL ID
    '''
    p[0] = (p[1], p[2])


def p_statement_cmd(p):
    #! COMMAND
    '''
    stmt : COMMAND ID
    '''
    p[0] = {p[1]: p[2]}
    print(p[0])


def p_statement_abort(p):
    #! ABORT
    '''
    stmt : ABORT
    '''
    p[0] = {p[1]: True}
    print(p[0])

def p_expr_appr(p):
    '''
    expr : APPROACH
    '''
    p[0] = (p[1], True)

def p_expr_speed(p):
    '''
    expr : SPEED INT
         | SPEED REAL
    '''
    p[0] = (p[1], p[2])

def p_discovery(p):
    #! DISCOVERY
    '''
    stmt : DISCOVERY COMMA expr
    '''
    arg = p[3]
    if not arg[0] in ['AREA', 'DISTANCE']: raise SyntaxError
    p[0] = {}
    p[0][p[1]] = arg
    print(p[0])

def p_error(p):
    if p:
        print("Syntax error at '%s'" % p.value)
    else:
        print("Syntax error at EOF")

import ply.yacc as yacc # type: ignore
parser = yacc.yacc()

def parse(data, debug=0):
    parser.error = 0
    p = parser.parse(data, debug=debug)
    if parser.error:
        return None
    return p

if __name__ == '__main__':
    # lex.runmain()
    while True:
        try:
            s = input('mission > ')
        except EOFError:
            break
        if not s:
            continue
        yacc.parse(s + '\n')
