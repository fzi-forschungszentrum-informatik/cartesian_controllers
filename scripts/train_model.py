#!/usr/bin/env python3
################################################################################
# Copyright 2024 FZI Research Center for Information Technology
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
################################################################################

# -----------------------------------------------------------------------------
# \file    train_model.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2024/01/23
#
# -----------------------------------------------------------------------------

import os
import sys
from cartesian_skill_controller.dataset import Dataset
from cartesian_skill_controller.model import Model

TYPE = "put"  # {put, get}
SEQ_LEN = 50


def main():
    training_files = os.path.join(os.getcwd(), f"../datasets/{TYPE}/train")
    evaluation_files = os.path.join(os.getcwd(), f"../datasets/{TYPE}/eval")
    training_data = Dataset(training_files, sequence_length=SEQ_LEN)
    evaluation_data = Dataset(evaluation_files, sequence_length=SEQ_LEN)
    model_dir = os.path.join(os.getcwd(), f"../models/{TYPE}")

    model = Model(n_nodes=64, key_dim=128, n_heads=4, sequence_length=SEQ_LEN)
    try:
        model.train(
            training_data,
            evaluation_data,
            epochs=10,
            iterations=1,
            batch_size=128,
            learning_rate=0.0005,
            log_dir=model_dir,
        )
    except KeyboardInterrupt:
        model.save(model_dir)
        print(f"Training canceled. Saved intermediate model here: {model_dir}")
        sys.exit()

    model.save(model_dir)
    print(f"Training finished. Saved model here: {model_dir}")


if __name__ == "__main__":
    main()
