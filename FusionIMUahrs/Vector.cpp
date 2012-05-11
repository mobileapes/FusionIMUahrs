/*
 * http://en.wikipedia.org/wiki/Cross_product
 * 
 * a_x = b_y * c_z - b_z * c_y
 * a_y = b_z * c_x - b_x * c_z
 * a_z = b_x * c_y - b_y * c_x
 * 
 */

void VectorCrossProduct(float a[3], float b[3], float c[3]) {
	a[0] = (b[1] * c[2]) - (b[2] * c[1]);
	a[1] = (b[2] * c[0]) - (b[0] * c[2]);
	a[2] = (b[0] * c[1]) - (b[1] * c[0]);
}
