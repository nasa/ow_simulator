/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 ******************************************************************************/
#ifndef ForceRow_h
#define ForceRow_h

#include "CSVRow.h"

class ForceRow
{
public:
  ForceRow(CSVRow& csvrow)
  {
    // TODO: add error checking
    std::stringstream(csvrow[0]) >> m_m;
    std::stringstream(csvrow[1]) >> m_d;
    std::stringstream(csvrow[2]) >> m_p;
    std::stringstream(csvrow[3]) >> m_rho;
    for(int i=0; i<6; i++) {
      float value;
      std::stringstream(csvrow[4 + i]) >> value;
      m_force_torque.push_back(value);
    }
  }

  int m_m;
  int m_d;
  int m_p;
  float m_rho;
  std::vector<float> m_force_torque;
};

#endif
