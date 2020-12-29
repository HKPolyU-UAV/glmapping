#include "cylindrical_cell.h"

cylindrical_cell::cylindrical_cell()
{

}

cylindrical_cell::cylindrical_cell(int Rho, int Phi, int Z, Vec3 vis_xyz)
{
    this->idx_rho = Rho;
    this->idx_phi = Phi;
    this->idx_z = Z;
    this->vis_pt=vis_xyz;
    this->landing_rate_to_sensor = static_cast<float>(Z)/static_cast<float>(Rho);

}
