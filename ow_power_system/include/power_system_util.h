#ifndef __POWER_SYSTEM_NODE_H__
#define __POWER_SYSTEM_NODE_H__

#include <vector>
#include <map>
#include <string>

#include "PrognoserFactory.h"

std::vector<std::map<MessageId, Datum<double>>> read_file(const std::string& filename);

#endif
