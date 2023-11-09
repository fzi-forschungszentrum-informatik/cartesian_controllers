## Python dependencies

```bash
pip3 install --user --upgrade scikit-learn scipy
pip3 install --user tensorflow==2.13.0 tensorflow-probability==0.21.0
```

## Run tests manually
In the root of a sourced workspace, call
```bash
colcon test --packages-select rackki_learning && colcon test-result --verbose
```
to run and inspect the integration tests locally.

You can clean the test results and outdated errors with
```bash
colcon test-result --delete-yes
```
