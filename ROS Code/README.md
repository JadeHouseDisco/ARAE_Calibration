# Steps to configure the ARAE into personalized mode
1. Set the weight to upper arm product (upper arm mass * upper arm COM) (Assumed that this value is less than 10)
2. Set the scale to forearm mass
3. Set the vertical load to forearm COM

Note that ARAE operates under personalized mode when the weight value is lower than 10.

# Files that were replaced
1. ULE_ARAE\src\ulexo_arm\api\ARAE_core\src\ARAE.cpp
2. ULE_ARAE\src\ulexo_arm\src\ulexo_arm.cpp