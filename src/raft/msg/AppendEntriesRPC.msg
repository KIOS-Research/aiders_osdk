uint32 term
string leaderId

#prevLogIndex index of log entry immediately preceding new ones
#prevLogTerm term of prevLogIndex entry
#entries[] log entries to store (empty for heartbeat;
#may send more than one for efficiency)
#leaderCommit leader’s commitIndex
