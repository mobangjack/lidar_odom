#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "lidar.h"
#include <math.h>
#include <float.h>

/**
 * @brief Polar2Rectangular Polar coordinate to rectangular coordinate transform.
 * @param theta Polar-theta.
 * @param rho Polar-rho.
 * @param x Rectangular-x.
 * @param y Rectangular-y.
 */
void Polar2Rectangular(float theta, float rho, float* x, float* y);

/**
 * @brief Rectangular2Polar Rectangular coordinate to polar coordinate transform.
 * @param x Rectangular-x.
 * @param y Rectangular-y.
 * @param theta Polar-theta.
 * @param rho Polar-rho.
 */
void Rectangular2Polar(float x, float y, float* theta, float* rho);

/**
 * @brief RTT Rotation-translation transform.
 * @param input Input point claster.
 * @param output Output point claster.
 * @param dim Claster dimension.
 * @param lambda Range resolution.
 * @param k Rotation offset.
 * @param l Translation offset.
 */
void RTT(
        float* input,
        float* output,
        int dim,
        float lambda,
        int k,
        int l
        );

/**
 * @brief CoordinateDistance Compute the distance between two polar points.
 * @param theta1 Polar theta of point no.1.
 * @param rho1 Polar rho of point no.1.
 * @param theta2 Polar theta of point no.2.
 * @param rho2 Polar rho of point no.2.
 * @return
 */
float CoordinateDistance(float theta1, float rho1, float theta2, float rho2);

/**
 * @brief ClasterCoordinateDistance Compute the distance between two point-clasters.
 * @param ref Reference claster.
 * @param obj Object claster.
 * @param dim Claster dimension.
 * @param omega Angle resolution.
 * @param lambda Range resolution.
 * @param k Angle offset.
 * @param l Range offset.
 * @return Geometry distance provided by euclidean metric.
 */
float ClasterCoordinateDistance(
        float* ref,
        float* obj,
        int dim,
        float omega,
        float lambda,
        int k,
        int l
        );

/**
 * @brief MinimizeClasterCoordinateDistance Minimize the distance between two point-clasters.
 * @param ref Reference claster.
 * @param obj Object claster.
 * @param dim Claster dimension.
 * @param omega Angle resolution.
 * @param lambda Range resolution.
 * @param k Angle offset to obtain.
 * @param kmin Minimum of angle offset.
 * @param kmax Maximum of angle offset.
 * @param l Range offset to obtain.
 * @param lmin Minimum of range offset.
 * @param lmax Maximum of range offset.
 * @return The minimum distance between the two point-clasters.
 */
float MinimumClasterCoordinateDistance(
        float* ref,
        float* obj,
        int dim,
        float omega,
        float lambda,
        int* k,
        int kmin,
        int kmax,
        int* l,
        int lmin,
        int lmax
        );

/**
 * @brief Adjacency Measure distance between two point in the same claster.
 * @param claster Point claster.
 * @param omega Angle resolution.
 * @param i Point index one of two.
 * @param j Point index two of two.
 * @return Distance between the two point.
 */
float Adjacency(
        float* claster,
        float omega,
        int i,
        int j);

/**
 * @brief ComputeAdjacencyVector Compute the adjacency vector of a point claster.
 * @param claster Point claster.
 * @param omega Angle resolution.
 * @param point Target point.
 * @param adj_vec The adjacency to obtain.
 * @param adj_dim The dimension of the adjacency vector.
 */
void ComputeAdjacencyVector(
        float* claster,
        float omega,
        int point,
        float* adj_vec,
        float adj_dim
        );

#endif
