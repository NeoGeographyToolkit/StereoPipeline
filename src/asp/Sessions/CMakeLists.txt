 

set(SP_SESSIONS_SRC_FILES 
        
        StereoSession.h
        StereoSession.cc
        
        DG/LinescanDGModel.h
        DG/StereoSessionDG.h
        DG/XMLBase.h
        DG/XML.h
        DG/StereoSessionDG.cc
        DG/XMLBase.cc
        DG/XML.cc
        
        DGMapRPC/StereoSessionDGMapRPC.h
        DGMapRPC/StereoSessionDGMapRPC.cc
        
        ISIS/PhotometricOutlier.h
        ISIS/PhotometricOutlier.cc
        ISIS/StereoSessionIsis.h
        ISIS/StereoSessionIsis.cc
        
        NadirPinhole/StereoSessionNadirPinhole.h
        NadirPinhole/StereoSessionNadirPinhole.cc
        
        Pinhole/StereoSessionPinhole.h
        Pinhole/StereoSessionPinhole.cc
        
        RPC/RPCModelGen.h
        RPC/RPCModel.h
        RPC/RPCStereoModel.h
        RPC/StereoSessionRPC.h
        RPC/RPCModel.cc
        RPC/RPCModelGen.cc
        RPC/RPCStereoModel.cc
        RPC/StereoSessionRPC.cc        
)

# Use wrapper function at this level to avoid code duplication
add_library_wrapper(SpSessions "${SP_SESSIONS_SRC_FILES}" "${SP_SESSIONS_TEST_FILES}" "${SP_SESSIONS_LIB_DEPENDENCIES}")


