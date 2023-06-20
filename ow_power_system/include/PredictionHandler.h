// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// This is the header file of the PredictionHandler class, which deals with
// the asynchronous callbacks from GSAP's prognosers during runtime.
// It handles the setup for subscribing to a provided MessageBus which returns
// GSAP predictions as they are completed, as well as the function that runs
// on receiving those predictions.

#ifndef __PREDICTION_HANDLER_H__
#define __PREDICTION_HANDLER_H__

#include <vector>
#include <map>

#include "Messages/MessageBus.h"
#include "Messages/ProgEventMessage.h"
#include "Messages/ScalarMessage.h"
#include "ModelBasedAsyncPrognoserBuilder.h"

// The PredictionHandler class subscribes to the battery EoD event message and
// prints each event as it is received.
class PredictionHandler : public IMessageProcessor
{
public:
  PredictionHandler(double& rul, double& soc, double& temp, MessageBus& bus,
                    const std::string& src, bool& bus_status);
  ~PredictionHandler();
  // Copy constructor & assignment operator. Neither should be allowed given
  // how PredictionHandler uses references.
  PredictionHandler(const PredictionHandler&) = delete;
  PredictionHandler& operator=(const PredictionHandler&) = delete;
  void processMessage(const std::shared_ptr<Message>& message) override;
private:
  double findMedian(std::vector<double> samples);

  // References used by other classes to process EoD predictions.
  double& m_rul_ref;
  double& m_soc_ref;
  double& m_temp_ref;
  MessageBus& m_bus;
  bool& m_bus_status;
};

#endif
