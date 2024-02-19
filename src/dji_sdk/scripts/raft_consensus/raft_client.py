import os
import sys
import datetime

import time
from threading import Thread, Lock

import rospy
from std_msgs.msg import String
import random
import copy

from raft.msg import AppendEntriesRPC, RequestVoteRPC

debug_messages = False

class RaftClient:
    def __init__(self, clientId):
        self.clientId = clientId            
        self.alive = False       

        # Client Pool
        self.client_pool = {}
        self.client_pool_previous = {}
        self.client_pool_mutex = Lock()
        self.clientPoolPub = rospy.Publisher('/raft/client_pool', String, queue_size=1)
        
        self.clientHeartbeatTimeoutS = 10
        self.clientHeartbeatRate = 1 / self.clientHeartbeatTimeoutS * 2
        
        self.keepalive_thread = Thread(target=self.update_client_pool)
        self.clientHeartbeatPub = rospy.Publisher('/raft/client_pool_heartbeat', String, queue_size=1)
        self.clientHeartbeatSub = rospy.Subscriber('/raft/client_pool_heartbeat', String, self.clientPoolHeartbeatCB)
        
        # Term & Leader
        self.term = 0
        self.term_mutex = Lock()

        self.leader = None
        self.leader_heartbeat_time = None
        self.leaderHeartbeatTimeoutS = 10
        self.leader_mutex = Lock()

        self.leaderHeartbeatPub = rospy.Publisher('/raft/raft_leader', AppendEntriesRPC, queue_size=1)
        self.leaderHeartbeatSub = rospy.Subscriber('/raft/raft_leader', AppendEntriesRPC, self.leaderHeartbeatCB)

        # Election
        self.voted_for = None
        self.votes = {}
        self.election_start_epoch = None
        self.election_started = False
        self.candidature_started = False
        self.election_timeout = 20
        self.vote_mutex = Lock()

        self.candidateVotePub = rospy.Publisher('/raft/raft_votes', RequestVoteRPC, queue_size=1)            
        self.leaderVotingSub = rospy.Subscriber('/raft/raft_votes', RequestVoteRPC, self.leaderVotingCB)

        # State Machine
        self.cur_state = 0
        self.state_dict = {0 : 'follower', 1 : 'candidate', 2 : 'leader'}
        self.state_mutex = Lock()
        
        # Epoch & Timing
        self.epoch_mutex = Lock()
        self.epochLength = 1 / 0.2
        self.nextEpoch = time.time() + self.epochLength
        self.epoch_rate = rospy.Rate(self.epochLength * 2)       
        self.state_machine_thread = Thread(target=self._state_machine)

    def clientPoolHeartbeatCB(self, heartbeat):
        if self.alive:
            clientId = heartbeat.data

            self.client_pool_mutex.acquire()

            if self.clientId != clientId:
                if self.state_dict[self.cur_state] == 'leader':
                    if not clientId in self.client_pool and not clientId in self.client_pool_previous:
                        print(self.clientId + ' - New drone(' + clientId + ') in client_pool, publishing leader heartbeat')
                        self.publish_leader_heartbeat()
                elif self.state_dict[self.cur_state] == 'candidate':
                    if not clientId in self.client_pool and not clientId in self.client_pool_previous:
                        print(self.clientId + ' - New drone(' + clientId + ') in client_pool, publishing candidate vote request')
                        self.publish_candidate_vote_request()

            self.client_pool[clientId] = time.time()
            
            self.client_pool_mutex.release()

    def update_client_pool(self):
        print(self.clientId + ' - Starting update_client_pool')
        heartbeat_rate = rospy.Rate(self.clientHeartbeatRate)
        
        while self.alive and not rospy.is_shutdown():
            heartbeat_rate.sleep()
            time_instance = time.time()

            self.client_pool_mutex.acquire()
            init_client_pool = copy.deepcopy(self.client_pool)
            clean_client_pool = copy.deepcopy(init_client_pool)
            self.client_pool_mutex.release()

            
            for client in init_client_pool.items():
                clientId = client[0]
                clientTime = client[1]

                time_diff = time_instance - clientTime
                if time_diff > self.clientHeartbeatTimeoutS:
                    del clean_client_pool[clientId]

            self.client_pool_mutex.acquire()
            self.client_pool_previous = copy.deepcopy(self.client_pool)
            self.client_pool = clean_client_pool
            self.client_pool_mutex.release()

            #print('Drones in Squadron:', clean_client_pool)

            if self.leader == self.clientId:
                client_pool_package = String()
                client_pool_package.data = str(clean_client_pool)
                self.clientPoolPub.publish(client_pool_package)
            
            self.publish_keepalive_heartbeat()


    def leaderHeartbeatCB(self, heartbeat):
        if self.alive:
            self.term_mutex.acquire()
            current_term = self.term
            self.term_mutex.release()

            if heartbeat.term >= current_term:
                if self.leader != heartbeat.leaderId and heartbeat.leaderId != self.clientId:
                    print(heartbeat.leaderId + ' - Elected Leader')

                self.leader_mutex.acquire()   
                self.leader = heartbeat.leaderId
                self.leader_heartbeat_time = time.time()
                self.leader_mutex.release()

                self.term_mutex.acquire()
                if heartbeat.term > self.term:
                    print(self.clientId + ' - Got bigger term from leader. Was', self.term, 'Now:', heartbeat.term)
                    self.term = heartbeat.term
                    self.term_mutex.release()

                    self.reset_epoch_length()
                else:     
                    self.term_mutex.release()

                if self.state_dict[self.cur_state] == 'candidate':
                    print(self.clientId + ' - Got message from leader while I was candidate. Demoted to follower')
                    self.cur_state = 0

                

                self.reset_election()

            else:
                self.term_mutex.acquire()
                current_term = self.term
                self.term_mutex.release()

                print(self.clientId + ' - Got message from leader but he is behind terms. Leader:', heartbeat.term, 'Me:', current_term)
                


    def leaderVotingCB(self, vote):
        if not vote.term < self.term and self.alive:
            self.epoch_mutex.acquire()
            current_term = self.term
            self.epoch_mutex.release()

            if current_term < vote.term:
                print(self.clientId + ' - Got newer term from candidate. Was', current_term, 'Now:', vote.term)
                self.updateTerm(vote.term)
            elif self.state_dict[self.cur_state] == 'leader':
                self.publish_leader_heartbeat()

            if self.state_dict[self.cur_state] != 'leader':
                self.vote_mutex.acquire()
                if not self.election_started and not self.election_start_epoch:
                    print(self.clientId + ' - Set election_start_epoch:', time.time())
                    self.election_started = True
                    self.election_start_epoch = time.time()
                self.vote_mutex.release()

                if self.state_dict[self.cur_state] == 'candidate':
                    print(self.clientId + ' - Got Vote:', vote)

                    self.vote_mutex.acquire()
                    self.votes[vote.clientId] = vote.candidateId
                    self.vote_mutex.release()
                elif self.state_dict[self.cur_state] == 'follower':
                    self.vote_mutex.acquire() 
                    voted_this_term = copy.deepcopy(self.voted_for)
                    self.vote_mutex.release()
                    
                    if not voted_this_term:
                        self.publish_candidate_vote(vote.candidateId)

    def start_threads(self):
        if not self.alive:
            print(self.clientId + ' - Starting up...')
            self.alive = True
            
            self.keepalive_thread = Thread(target=self.update_client_pool)
            self.state_machine_thread = Thread(target=self._state_machine)
            
            self.keepalive_thread.start()
            self.state_machine_thread.start()
        else:
            print(self.clientId + ' - Threads already running')

    def stop_threads(self):
        print(self.clientId + ' - Shutting down...')
        self.alive = False

        self.keepalive_thread.join()
        self.state_machine_thread.join()

    def _state_machine(self):
        self.epoch_mutex.acquire()
        self.initialEpoch = time.time()
        self.latestEpoch = self.initialEpoch

        self.epochLength = 1 / 0.2
        self.nextEpoch = time.time() + self.epochLength
        self.epoch_mutex.release()

        while self.alive and not rospy.is_shutdown():
            currentEpoch = time.time()

            self.epoch_mutex.acquire()
            if currentEpoch > self.nextEpoch:
                self.nextEpoch = self.nextEpoch + self.epochLength
                self.epoch_mutex.release()
                self.state_machine_step()
            else:
                self.epoch_mutex.release()

            if rospy.is_shutdown():
                self.alive = False

            self.epoch_rate.sleep()
    
    def advanceTerm(self):
        self.term_mutex.acquire()
        self.term = self.term + 1
        self.term_mutex.release()

        self.vote_mutex.acquire() 
        self.voted_for = False
        self.vote_mutex.release() 

    def updateTerm(self, new_term):
        self.term_mutex.acquire()
        self.term = new_term
        self.term_mutex.release()

        self.state_mutex.acquire()
        self.cur_state = 0
        self.state_mutex.release()

        self.epoch_mutex.acquire()
        self.nextEpoch = time.time() + self.epochLength
        self.epoch_mutex.release()

        self.vote_mutex.acquire() 
        self.voted_for = False
        self.vote_mutex.release() 

    def state_machine_step(self):
        currentEpoch = time.time()
        #print(self.clientId + ' - state_machine_step - Term:', self.term, '- State:', self.state_dict[self.cur_state]) 

        if self.state_dict[self.cur_state] == 'follower':
            self.follower_step(currentEpoch)            
        elif self.state_dict[self.cur_state] == 'candidate':
            self.candidate_step(currentEpoch)
        elif self.state_dict[self.cur_state] == 'leader':
            self.leader_step(currentEpoch)

        #print(self.clientId + ' - state_machine_step - Time Diff:', currentEpoch - self.latestEpoch)
        self.epoch_mutex.acquire()
        self.latestEpoch = currentEpoch
        self.epoch_mutex.release()

    def follower_step(self, currentEpoch):
        leader_timeout = False
        
        self.leader_mutex.acquire()
        if self.voted_for and self.election_start_epoch:
            #self.election_start_epoch = time.time()
            timediff = currentEpoch - self.election_start_epoch
            if timediff > self.election_timeout:
                self.reset_leader()
                self.reset_election()
            else:
                leader_timeout = False
                print(self.clientId + ' - Follower Voted for. Election still in process', str(timediff) + '/' + str(self.election_timeout))
        else:
            if self.leader and self.leader_heartbeat_time:
                if currentEpoch - self.leader_heartbeat_time > self.leaderHeartbeatTimeoutS:
                    leader_timeout = True
                else:
                    if debug_messages:
                        print(self.clientId + ' - Follower Step. Got Heartbeat. Leader:', self.leader, 'Time diff:', str(currentEpoch - self.leader_heartbeat_time))
            else:
                leader_timeout = True
            
            if leader_timeout:
                if random.random() > 0.5 or len(self.client_pool) < 2:
                    print(self.clientId + ' - Leader timeout. Promoted to candidate')
                    self.cur_state = 1
                    self.leader = None
                    self.leader_heartbeat_time = None
                else:   
                    print(self.clientId + ' - Leader timeout. Trying again')

        self.leader_mutex.release()
                

    def candidate_step(self, currentEpoch):
        if not self.candidature_started:
            self.start_canditate()
        elif self.candidature_started:
            self.finish_canditate()

    def leader_step(self, currentEpoch):
        self.publish_leader_heartbeat()


    def publish_keepalive_heartbeat(self):
        heartbeatPackage = String()
        heartbeatPackage.data = self.clientId       
        self.clientHeartbeatPub.publish(heartbeatPackage)

    def publish_leader_heartbeat(self):
        heartbeatPackage = AppendEntriesRPC()
        heartbeatPackage.term = self.term      
        heartbeatPackage.leaderId = self.clientId
        self.leaderHeartbeatPub.publish(heartbeatPackage)

    def get_leader_heartbeat(self):
        pass

    def start_canditate(self):
        print(self.clientId + ' - Starting canditature')
        self.candidature_started = True
        if not self.election_started:
            self.advanceTerm()
        
        self.publish_candidate_vote_request()

    def publish_candidate_vote_request(self):
        self.publish_candidate_vote(self.clientId)

    def publish_candidate_vote(self, candidateId):
        self.vote_mutex.acquire() 
        self.voted_for = candidateId
        self.vote_mutex.release()

        vote_package = RequestVoteRPC()
        vote_package.term = self.term
        vote_package.candidateId = candidateId
        vote_package.clientId = self.clientId
        self.candidateVotePub.publish(vote_package)

    def finish_canditate(self):
        print(self.clientId + ' - Election finished. Tallying votes')
        self.client_pool_mutex.acquire()
        client_pool = copy.deepcopy(self.client_pool)
        self.client_pool_mutex.release()

        self.vote_mutex.acquire()
        vote_list = copy.deepcopy(self.votes)
        self.vote_mutex.release()

        client_count = len(client_pool)
        vote_count = len(vote_list)
        
        elected = False

        if client_count < 2:
            print(self.clientId + ' - client count < 2. Skipping tallying')
            elected = True
        else:
            vote_tally = {}
            for vote in vote_list.values():
                if vote not in vote_tally:
                    vote_tally[vote] = 1
                else:
                    vote_tally[vote] = vote_tally[vote] + 1

            print(self.clientId + ' - vote_tally:', vote_tally, 'Voted:', str(vote_count) + '/' + str(client_count))

            if self.clientId in vote_tally:
                if vote_tally[self.clientId] > client_count / 2:
                    elected = True

        if elected:
            print(self.clientId + ' - Elected Leader')
            self.cur_state = 2
            self.publish_leader_heartbeat()
        else:
            if random.random() > 0.5:
                print(self.clientId + ' - Did not win Leader. Demoted to Follower')
                self.cur_state = 0
                self.candidature_started = False
            else:
                print(self.clientId + ' - Did not win Leader. Trying again')
                self.candidature_started = False

    def reset_leader(self):
        self.leader_mutex.acquire()
        self.leader = None
        self.leader_heartbeat_time = None
        self.leader_mutex.release()

    def reset_election(self):
        #print(self.clientId + ' - Election Timeout')

        self.vote_mutex.acquire()
        self.votes = {}
        self.voted_for = None
        
        self.election_start_epoch = None
        self.election_started = False
        self.candidature_started = False
        self.vote_mutex.release()

    def reset_epoch_length(self):
        current_time = time.time()
        self.epoch_mutex.acquire()
        self.nextEpoch = current_time + self.epochLength               
        self.epoch_mutex.release()

    def get_state(self):
        return self.state_dict[self.cur_state]





def listener(dji_name = "matrice300"):
    global boardId    

    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]
    nodename = dronename.replace('/', '') + '_raft'

    print('Raft Client', dronename, boardId)
    print('Raft Client', nodename)
    
    rospy.init_node(nodename)

    client = RaftClient(boardId)
    client.start_threads()
    #time.sleep(20)
    #client.stop_threads()
    #client.start_threads()

    #client = RaftClient(boardId + '_1')
    #client.start_threads()
    if False:
        time.sleep(15)

        client_pool = []

        for i in range(3):
            client_pool.append(RaftClient(boardId + '_' + str(i)))
            client_pool[i].start_threads()

        time.sleep(30)
        client.stop_threads()
        
        while len(client_pool) > 1:
            time.sleep(30)
            print('client_pool size:', len(client_pool))
            for i in range(len(client_pool)):
                if client_pool[i].get_state() == 'leader':
                    client_pool[i].stop_threads()
                    del client_pool[i]
                    break
    
    rospy.spin()




if __name__=='__main__': 
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    print('Initializing Raft Client... (' + str(boardId) + ')')
    
    dronename = '/matrice300' + '_' + boardId
    listener(dronename)