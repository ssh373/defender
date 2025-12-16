#include "brain_config.h"
#include "utils/print.h"


void BrainConfig::handle()
{
    // playerRole [striker, goal_keeper]
    if (playerRole != "striker" && playerRole != "goal_keeper")
    {
        throw invalid_argument("player_role must be one of [striker, goal_keeper]. Got: " + playerRole);
    }

    // playerId
    if (playerId < 1 || playerId > HL_MAX_NUM_PLAYERS)
    {
        throw invalid_argument("[Error] player_id must be one of [1, .. 11]. Got: " + to_string(playerId));
    }

    // fieldType [adult_size, kid_size]
    if (fieldType == "adult_size")
    {
        fieldDimensions = FD_ADULTSIZE;
    }
    else if (fieldType == "kid_size")
    {
        fieldDimensions = FD_KIDSIZE;
    }
    else if (fieldType == "robo_league")
    {
        fieldDimensions = FD_ROBOLEAGUE;
    }
    else
    {
        throw invalid_argument("[Error] fieldType must be one of [adult_size, kid_size, robo_league]. Got: " + fieldType);
    }
}

