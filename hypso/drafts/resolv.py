from sympy import symbols, sin, tan, cos, Eq, solve

''' 
For now it is just an example in case I need to solve a difficult equation and get a formula to implement something
'''

# Définir les variables symboliques
a, b, c, d, e = symbols('a b c d e')

# Équation : sin(a + b) = tan(c) * cos(d - e)
eq = Eq(sin(a + b), tan(c) * cos(d - e))

# Résoudre pour e
solutions = solve(eq, e)

# Afficher les solutions
for sol in solutions:
    print(f"e = {sol}")
