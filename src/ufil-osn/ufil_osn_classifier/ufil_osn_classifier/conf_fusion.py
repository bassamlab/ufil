# Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import numpy as np


# Class to store confidence values
class ConfidenceStorage:
    def __init__(self):
        self.storage = []  # Initialize an empty list to store confidence values
        pass

    def add_value(self, id, conf):
        """
        Add a new confidence value to the storage.
        """
        self.storage = np.append(self.storage, ConfidenceValue(id, conf))
        pass

    def get_id(self, conf):
        """
        Get the ID of a confidence value.
        """
        return conf.id
    
    def show(self):
        """
        Display all confidence values in the storage.
        """
        for value in self.storage: 
            value.show()
        pass
    
    def check_for_id(self, id):
        """
        Check if a confidence value with the given ID exists in the storage.
        """
        for conf in self.storage:
            if conf.id == id:
                return conf
        return False
    
    def fuse_log(self, new_conf, old_log, alpha):
        """
        Fuse new confidence values with old log values using a specified alpha.
        """
        eps = np.finfo(float).eps  # Small value to prevent division by zero
        alpha = alpha 
        updated_log = []
        log_prior = np.log((1/7)/(1-(1/7)))
        for i in range(7): 
            new_conf.set_log_conf(i, max(min(new_conf.log_conf[i], 1-eps), eps))
            new_log = np.log(new_conf.log_conf[i] / (1 - new_conf.log_conf[i]))
            updated_log = (new_log - log_prior + alpha * old_log.log_conf[i])
            old_log.set_log_conf(i, updated_log)
            old_log.seen()
        return old_log
    
    def log_to_conf(self, logs):
        """
        Convert log values to confidence values.
        """
        bel = 1-(1/(1+np.exp(logs.log_conf)))
        return bel / np.sum(bel)
    
    def call_fusion(self,id,conf, alpha):
        old_log = self.check_for_id(id)
        if not old_log:
            self.add_value(id, np.full(7, 1/7))
            old_log = self.check_for_id(id)
        for i in range(7):
            fused_conf = alpha * conf[i] + (1-alpha)* old_log.log_conf[i]
            old_log.set_log_conf(i, fused_conf)
            old_log.seen()
        return old_log.log_conf
    
    def keep_fusion(self, id):
        """
        Keep the existing fusion for a given ID.
        """
        old_log = self.check_for_id(id)
        if not old_log:
            l_0 = np.log((1-1/7)/(1/7))
            self.add_value(id, np.full(7,l_0))
            old_log = self.check_for_id(id)
            print("ERROR in keep_fusion()!! Hier sollte man nicht hin kommen können!!")
        else: 
            old_log.seen()
            return old_log.log_conf

    def kill_expred(self):
        """
        Remove expired entries from the storage.
        """
        for conf in self.storage:
            if not conf.was_seen:
                self.storage = np.delete(self.storage, self.storage == conf)
            else: 
                conf.reset_seen()
        pass

class ConfidenceValue:
    def __init__(self, id, log_conf):
        self.id = id  # ID of the confidence value
        self.log_conf = log_conf  # Log confidence values
        self.was_seen = False  # Flag to indicate if the confidence value was seen

    def set_log_conf(self, index, value):
        """
        Set the log confidence value at a specific index.
        """
        self.log_conf[index] = value
        pass

    def seen(self):
        """
        Mark the confidence value as seen.
        """
        self.was_seen = True
        pass
    
    def reset_seen(self):
        """
        Reset the seen flag for the confidence value.
        """
        self.was_seen = False
        pass

    def show(self):
        """
        Display the confidence value.
        """
        print(f"id :  {self.id}")
        print(f" log_conf: {self.log_conf}")
        pass


def conf_fusion(confs, tracker_trust, camera_trust, track_conf = [None]):
    fused_conf = np.zeros(7)
    if len(track_conf) < 7:
        for conf in confs: 
            fused_conf += conf * (1/len(confs))
    else: 
        for conf in confs: 
            fused_conf += conf * camera_trust
        fused_conf += track_conf * tracker_trust
    return fused_conf / sum(fused_conf)