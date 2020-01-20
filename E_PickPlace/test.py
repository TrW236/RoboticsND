from sympy import *


# Create symbols
d1,     d2,     d3,     d4,     d5,     d6,     d7      = symbols('d1:8')
a0,     a1,     a2,     a3,     a4,     a5,     a6      = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6  = symbols('alpha0:7')
q1,     q2,     q3,     q4,     q5,     q6,     q7      = symbols('q1:8')

# Create Modified DH parameters
DH_table = {
    alpha0: 0, 	        a0: 0, 		d1: 0.75, 	q1: q1,
    alpha1: -pi / 2.,   a1: 0.35,	d2: 0, 		q2: -pi / 2. + q2,
    alpha2: 0, 	        a2: 1.25, 	d3: 0, 		q3: q3,
    alpha3: -pi/2.,     a3: -0.054, d4: 1.5, 	q4: q4,
    alpha4: pi/2, 	    a4: 0, 		d5: 0, 		q5: q5,
    alpha5: -pi/2.,     a5: 0, 		d6: 0, 		q6: q6,
    alpha6: 0, 	        a6: 0, 		d7: 0.303, 	q7: 0
    }
# Define Modified DH Transformation matrix
def get_transformation_matrix(alpha, a, d, q):
    TFMat = Matrix([
        [cos(q), 		        -sin(q), 		        0.0, 		    a               ],
        [sin(q) * cos(alpha), 	cos(q) * cos(alpha), 	-sin(alpha), 	-sin(alpha) * d ],
        [sin(q) * sin(alpha), 	cos(q) * sin(alpha), 	cos(alpha), 	cos(alpha) * d  ],
        [0.0,			        0.0,			        0.0,		    1.0             ]
        ])
    return TFMat

# Create individual transformation matrices
TFMat_0t1 = get_transformation_matrix(alpha0, a0, d1, q1).subs(DH_table)
TFMat_1t2 = get_transformation_matrix(alpha1, a1, d2, q2).subs(DH_table)
TFMat_2t3 = get_transformation_matrix(alpha2, a2, d3, q3).subs(DH_table)
TFMat_3t4 = get_transformation_matrix(alpha3, a3, d4, q4).subs(DH_table)
TFMat_4t5 = get_transformation_matrix(alpha4, a4, d5, q5).subs(DH_table)
TFMat_5t6 = get_transformation_matrix(alpha5, a5, d6, q6).subs(DH_table)
TFMat_6tE = get_transformation_matrix(alpha6, a6, d7, q7).subs(DH_table)

TFMat_0tE = simplify(TFMat_0t1 * TFMat_1t2 * TFMat_2t3 * TFMat_3t4 * TFMat_4t5 * TFMat_5t6 * TFMat_6tE)

print(TFMat_0tE.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))

