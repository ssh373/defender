#include "brain_data.h"
#include "utils/math.h"

BrainData::BrainData()
{
    std::fill(std::begin(penalty), std::end(penalty), SUBSTITUTE);
}
