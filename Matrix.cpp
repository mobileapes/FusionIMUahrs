/*
 * Multiply two 3x3 matrixs. This function developed by Jordi can be
 * easily adapted to multiple n*n matrix's. (Pero me da flojera!).
 */

void MatrixMultiply(float a[3][3], float b[3][3], float c[3][3]) {
	float op[3]; 
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			for(int k=0; k<3; k++)
				op[k] = a[i][k] * b[k][j];
			c[i][j] = 0.0f;
			c[i][j] = op[0] + op[1] + op[2];
		}
	}
}

/*
 * slow algorithm
 *
void MatrixMultiply(float a[3][3], float b[3][3], float c[3][3]) {
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            c[i][j] = 0.0;
            for(int k=0; k<3; k++) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}
*/

void MatrixAdition(float a[3][3], float b[3][3]) {
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
			a[i][j] += b[i][j];
}

/*
void MatrixAdition(float a[3][3], float b[3][3]) {
    a[0][0] = a[0][0] + b[0][0];
    a[0][1] = a[0][1] + b[0][1];
    a[0][2] = a[0][2] + b[0][2];

    a[1][0] = a[1][0] + b[1][0];
    a[1][1] = a[1][1] + b[1][1];
    a[1][2] = a[1][2] + b[1][2];

    a[2][0] = a[2][0] + b[2][0];
    a[2][1] = a[2][0] + b[2][1];
    a[2][2] = a[2][0] + b[2][2];
}
*/
