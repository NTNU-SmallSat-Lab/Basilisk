# def setupAttitudeControl(scSim, simTaskName, scObject, att_ref_msg_obj):
    #     """Configure le système de contrôle d'attitude pour un vaisseau spatial
        
    #     Args:
    #         scSim: Simulation object
    #         simTaskName: Nom de la tâche de simulation
    #         scObject: Objet vaisseau spatial
    #         att_ref_msg_obj: Message de référence d'attitude
    #     """
        
    #     # 1. Configuration du module de navigation
    #     nav = simpleNav.SimpleNav()
    #     nav.ModelTag = "SimpleNav"
    #     scSim.AddModelToTask(simTaskName, nav)
    #     nav.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
        
    #     # 2. Création du message de configuration du véhicule
    #     vehicleConfigOutMsg = messaging.VehicleConfigMsgPayload()
        
    #     # Détermination de la matrice d'inertie selon la version de Basilisk
    #     if hasattr(scObject.hub, 'I_sc'):  # Ancienne version
    #         I = scObject.hub.I_sc
    #     elif hasattr(scObject.hub, 'inertia'):  # Nouvelle version
    #         I = scObject.hub.inertia
    #     elif hasattr(scObject.hub, 'IHubPntBc_B'):  # Version alternative
    #         I = scObject.hub.IHubPntBc_B
    #     else:
    #         I = [[1000.0, 0.0, 0.0],  # Valeurs par défaut
    #             [0.0, 800.0, 0.0],
    #             [0.0, 0.0, 800.0]]
        
    #     # Configuration de la matrice d'inertie
    #     vehicleConfigOutMsg.ISCPntB_B = [
    #         I[0][0], I[1][1], I[2][2],  # Diagonale principale
    #         I[0][1], I[0][2], I[1][2]   # Termes de couplage
    #     ]
        
    #     # Masse et centre de masse
    #     vehicleConfigOutMsg.mass = getattr(scObject.hub, 'mHub', 100.0)  # kg
    #     vehicleConfigOutMsg.CoB_B = getattr(scObject.hub, 'r_BcB_B', [0.0, 0.0, 0.0])
        
    #     vcMsg = messaging.VehicleConfigMsg_C().write(vehicleConfigOutMsg)
        
    #     # 3. Module de calcul d'erreur d'attitude
    #     attError = attTrackingError.attTrackingError()
    #     attError.ModelTag = "AttError"
    #     scSim.AddModelToTask(simTaskName, attError)
    #     attError.attRefInMsg.subscribeTo(att_ref_msg_obj)
    #     attError.attNavInMsg.subscribeTo(nav.attOutMsg)
        
    #     # 4. Contrôleur MRP
    #     mrpControl = mrpFeedback.mrpFeedback()
    #     mrpControl.ModelTag = "mrpFeedback"
    #     scSim.AddModelToTask(simTaskName, mrpControl)
    #     mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    #     mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
        
    #     # Paramètres du contrôleur (valeurs typiques)
    #     mrpControl.K = 3.5      # Gain proportionnel
    #     mrpControl.Ki = -1.0    # Gain intégral (mettre à 0 pour désactiver)
    #     mrpControl.P = 30.0     # Gain dérivé
        
    #     # 5. Application du couple sur le vaisseau
    #     extFTObject = extForceTorque.ExtForceTorque()
    #     extFTObject.ModelTag = "externalDisturbance"
    #     extFTObject.extTorquePntB_B = [0., 0., 0.]  # Perturbations externes
    #     scObject.addDynamicEffector(extFTObject)
    #     scSim.AddModelToTask(simTaskName, extFTObject)
    #     extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)

    # # Appelez cette fonction avant l'initialisation
    # setupAttitudeControl(scSim, simTaskName, hypso, att_ref_msg_obj)
    


    # if hasattr(hypso.hub, 'inertia'):
    #     hypso.hub.inertia = [[1000,0,0],[0,800,0],[0,0,800]]
    # elif hasattr(hypso.hub, 'I_sc'):
    #     hypso.hub.I_sc = [[1000,0,0],[0,800,0],[0,0,800]]











# nav = simpleNav.SimpleNav()
#         nav.ModelTag = "simpleNav"
#         scSim.AddModelToTask(simTaskName, nav)

#         nav.scStateInMsg.subscribeTo(hypso.scStateOutMsg)
#         nav_att_msg = nav.attOutMsg







    # # setup extForceTorque module
    #     # the control torque is read in through the messaging system
    #     extFTObject = extForceTorque.ExtForceTorque()
    #     extFTObject.ModelTag = "externalDisturbance"
    #     # use the input flag to determine which external torque should be applied
    #     # Note that all variables are initialized to zero.  Thus, not setting this
    #     # vector would leave it's components all zero for the simulation.
    #     hypso.addDynamicEffector(extFTObject)
    #     scSim.AddModelToTask(simTaskName, extFTObject)

    #     # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    #     # velocity navigation message
    #     sNavObject = simpleNav.SimpleNav()
    #     sNavObject.ModelTag = "SimpleNavigation"
    #     scSim.AddModelToTask(simTaskName, sNavObject)

    #     #
    #     #   setup the FSW algorithm tasks
    #     #

    #     # setup inertial3D guidance module
    #     inertial3DObj = inertial3D.inertial3D()
    #     inertial3DObj.ModelTag = "inertial3D"
    #     scSim.AddModelToTask(simTaskName, inertial3DObj)
    #     inertial3DObj.sigma_R0N = Sigma # set the desired inertial orientation

    #     # setup the attitude tracking error evaluation module
    #     attError = attTrackingError.attTrackingError()
    #     attError.ModelTag = "attErrorInertial3D"
    #     scSim.AddModelToTask(simTaskName, attError)

    #     # setup the MRP Feedback control module
    #     mrpControl = mrpFeedback.mrpFeedback()
    #     mrpControl.ModelTag = "mrpFeedback"
    #     scSim.AddModelToTask(simTaskName, mrpControl)
    #     mrpControl.K = 3.5
    #     mrpControl.Ki = -1  # make value negative to turn off integral feedback
    #     mrpControl.P = 30.0
    #     mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    #     #
    #     # create simulation messages
    #     #
    #     # The MRP Feedback algorithm requires the vehicle configuration structure. This defines various spacecraft
    #     # related states such as the inertia tensor and the position vector between the primary Body-fixed frame
    #     # B origin and the center of mass (defaulted to zero).  The message payload is created through
    #     configData = messaging.VehicleConfigMsgPayload()
    #     configData.ISCPntB_B = [1., 0., 0.,
    #                             0., 1., 0.,
    #                             0., 0., 1.]
    #     # Two methods are shown to create either a C++ or C wrapped msg object in python.  The
    #     # preferred method is to just create C++ wrapped messages.

    #     configDataMsg = messaging.VehicleConfigMsg()
        
    #     configDataMsg.write(configData)

    #     #
    #     # connect the messages to the modules
    #     #
    #     sNavObject.scStateInMsg.subscribeTo(hypso.scStateOutMsg)
    #     attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    #     attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    #     mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    #     mrpControl.vehConfigInMsg.subscribeTo(configDataMsg)
            
    #     # connect to module-internal commanded torque msg
    #     extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)