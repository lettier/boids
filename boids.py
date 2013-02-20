##############################################################################
#                                                                            #
# boids.py                                                                   #
#                                                                            #
# Autonomous Steering Behaviors: Seek & Arrive                               #
#                                                                            #
# Special thanks to Daniel Shiffman. http://www.shiffman.net/                #
#                                                                            #
# http://www.dldownload.info/                                                #
#                                                                            #
# Created with Panda3D version 1.3.2, may need this version for py to work.  #
#                                                                            #
# Copyright 2006 David Lettier.                                              #
#                                                                            #
##############################################################################
try:
    import sys, os, math, random, time
    from pandac.PandaModules                 import AmbientLight, DirectionalLight, PointLight
    from pandac.PandaModules                 import LightAttrib
    from direct.gui.DirectButton             import DirectButton
    from pandac.PandaModules                 import Vec3, Vec4, Point3, BitMask32, Plane
    from pandac.PandaModules                 import VBase4
    from pandac.PandaModules                 import AudioManager
    from pandac.PandaModules                 import *
    from direct.showbase                     import Audio3DManager
    from direct.actor.Actor                  import Actor
    from direct.gui.OnscreenText             import OnscreenText
    from direct.showbase.DirectObject        import DirectObject
    from direct.gui.DirectGui                import *
    from pandac.PandaModules                 import CollisionTraverser, CollisionNode
    from pandac.PandaModules                 import CollisionHandlerQueue, CollisionRay, CollisionPlane
    import direct.directbase.DirectStart    
    # end try 
except ImportError, err:
    # we have an import error
    print "ImportError: %s." % ( err ) # print the error    
    sys.exit( 1 ) # exit the program    
    # end except
#
class cBoid:
    def __init__( self ):
        # 3d vector for location
        self.location = Vec3()
        # 3d vector for velocity
        self.velocity = Vec3()
        # 3d vector for acceleration
        self.acceleration = Vec3()
        # float for radius
        self.radius = 2.0
        # float for max force
        self.maxForce = 0.0
        # float max speed
        self.maxSpeed = 0.0
    def setup( self, modelSrc, location, maxForce, maxSpeed ):
        # load in model file
        self.boidModel = loader.loadModel( modelSrc )
        # parent
        self.boidModel.reparentTo( render ) 
        # set location
        self.boidModel.setPos( location )
        # set max force / speed
        self.maxForce = maxForce
        self.maxSpeed = maxSpeed
    def update( self ):
        # update velocity
        self.velocity = self.velocity + self.acceleration
        # limit speed
        if ( self.velocity.length( ) > self.maxSpeed ):
            self.velocity.normalize( )
            self.velocity = self.velocity * self.maxSpeed
        # add velocity vector to location vector
        self.location = self.location + self.velocity
        # update boid model position in 3d space
        self.boidModel.setPos( self.location )
        # reset acceleration to zero
        self.acceleration.fill( 0.0 )
    def seek( self, target ):
        # add steer vector to acceleration vector
        self.acceleration = self.acceleration + self.steer( target, 0 )
        # look at target
        self.boidModel.lookAt( Point3( target ) ) 
    def arrive( self, target ):
        # add steer vector to acceleration vector
        self.acceleration = self.acceleration + self.steer( target, 1 )
        # look at target
        self.boidModel.lookAt( Point3( target ) ) 
    def steer( self, target, slowDown ):
        # create steering vector
        steer = Vec3( )
        # create desired vector by substracting target's location vector
        # from location vector
        desired = target - self.location
        # get distance/magnitude of desired vector
        distance = desired.length( )
        # if distance/magnitude is creater than one
        if ( distance > 1.0 ):
            # normalize desired vector
            desired.normalize( ) 
            # if we are arriving we need to slow down as we approach
            if ( ( slowDown > 0 ) and ( distance < 100.0 ) ):
                # multiply desired vector by ( max speed * ( distance / 100.0 ) )
                desired = desired * ( self.maxSpeed * ( distance / 100.0 ) )
            # else
            else: # we are not arriving so full speed ahead
                # multiply desired vector by max speed
                desired = desired * self.maxSpeed
            # steering vector is equal to desired vector minus velocity vector
            steer = desired - self.velocity
            # limit steering vector to max force
            if ( steer.length( ) > self.maxForce ):
                steer.normalize( ) # normalize. set magnitude to one.
                steer = steer * self.maxForce # multiply the manitude (=1) 
                                              # with max force
                                              # thus, manitude is = max force
        # else
        else:
            # set steering vector to zero or null
            steer = Vec3( )
            steer.fill( 0.0 )
        # return steering vector
        return steer
    def run( self ):
        self.update( ) # run update method
#
class cWorld:
    def __init__( self ):
        #  set background color
        base.setBackgroundColor( 0, 0, 0 ) 
        # create target
        self.createTarget( )
        # create boids
        self.createBoids( )
        # setup camera
        self.setupCamera( )
        # setup lights
        self.setupLights( )
        # setup collision detection
        self.setupCollision( )
        # add task
        taskMgr.add( self.steer, 'steer' ) # steer task
        taskMgr.add( self.moveTarget, 'moveTarget' ) # mouse move target task
    def createBoids( self ):
        self.redBoid = cBoid( ) # create red boid
        # setup blue boid with model path, starting location, max force, and max speed
        self.redBoid.setup( 'assets/models/boid_one.egg', Vec3( 0.0, 0.0, 0.0 ), 4.0, 0.1 )
        # create blue boid
        self.blueBoid = cBoid( )
        # setup blue boid with model path, starting location, max force, and max speed
        self.blueBoid.setup( 'assets/models/boid_two.egg', Vec3( 0.0, 0.0, 0.0 ), 4.0, 1.0 )
    def createTarget( self ):
        # load in model file
        self.target = loader.loadModel( 'assets/models/target.egg' )
        # parent
        self.target.reparentTo( render ) 
        # set location
        self.target.setPos( Vec3( 0.0, 0.0, 0.0 ) )
    def setupCamera( self ):
        # disable auto controls
        base.disableMouse()
        # set position, heading, pitch, and roll
        camera.setPosHpr( Vec3( 0.0, -45.0, 45.0), Vec3( 0.0, -45.0, 0 ) )
    def setupLights( self ):
        # create a point light
        plight = PointLight( 'plight' )
        # set its color 
        plight.setColor( VBase4( 1.0, 1.0, 1.0, 1 ) )
        # attach the light to the render 
        plnp = render.attachNewNode( plight.upcastToPandaNode( ) )
        # set position
        plnp.setPos( 0.0, 0.0, 2.0 )
        # turn on light 
        render.setLight( plnp )
    def setupCollision( self ):
        # create collision traverser
        self.picker = CollisionTraverser( )
        # create collision handler
        self.pq = CollisionHandlerQueue( )
        # create collision node
        self.pickerNode = CollisionNode( 'mouseRay' ) # create collision node
        # attach new collision node to camera node
        self.pickerNP = camera.attachNewNode( self.pickerNode ) # attach collision node to camera
        # set bit mask to one 
        self.pickerNode.setFromCollideMask( BitMask32.bit( 1 ) ) # set bit mask
        # create a collision ray
        self.pickerRay = CollisionRay( ) # create collision ray
        # add picker ray to the picker node 
        self.pickerNode.addSolid( self.pickerRay ) # add the collision ray to the collision node
        # make the traverser know about the picker node and its even handler queue
        self.picker.addCollider( self.pickerNP, self.pq ) # add the colision node path and collision handler queue
        #self.picker.showCollisions( render ) # render or draw the collisions
        #self.pickerNP.show( ) # render picker ray
        # create col node
        self.colPlane = CollisionNode( 'colPlane' )
        # add solid to col node plane
        self.colPlane.addSolid( CollisionPlane( Plane( Vec3( 0, 0, 1 ), Point3( 0, 0, 0 ) ) ) )
        # attach new node to the render
        self.colPlanePath = render.attachNewNode( self.colPlane )
        #self.colPlanePath.show( ) # render node
        # make the col plane look at the camera
        # this makes it alway look at the camera no matter the orientation
        # we need this because the ray nees to intersect a plane parallel
        # to the camera
        self.colPlanePath.lookAt( camera )
        # prop up the col plane
        self.colPlanePath.setP( -45 )
        # set bit mask to one
        # as I understand it, this makes all col nodes with bit mask one
        # create collisions while ignoring others of other masks
        self.colPlanePath.node( ).setIntoCollideMask( BitMask32.bit( 1 ) )
    def steer( self, Task ):
        # seek after target
        self.redBoid.seek( Vec3( self.target.getPos( ) ) )
        # run the algorithm
        self.redBoid.run( )
        # arrive at the target
        self.blueBoid.arrive( Vec3( self.target.getPos( ) ) )
        # run the algorithm
        self.blueBoid.run( )
        return Task.cont # continue task
    def moveTarget( self, Task ):
        # traverse through the render tree
        self.picker.traverse( render )
        # go through the queue of collisions 
        for i in range( self.pq.getNumEntries( ) ): 
            entry = self.pq.getEntry( i ) # get entry
            surfacePoint = entry.getSurfacePoint( render ) # get surface point of collision
            self.target.setPos( surfacePoint ) # set surface point to target's position 
        if base.mouseWatcherNode.hasMouse( ): # if we have a mouse
            mpos = base.mouseWatcherNode.getMouse( ) # get the path to the mouse 
            # shoot ray from camera
            # based on X & Y coordinate of mouse
            self.pickerRay.setFromLens( base.camNode, mpos.getX( ), mpos.getY( ) )
        return Task.cont # continue task
#
class cApplication( DirectObject ):
    def __init__( self ):
        # create world
        self.world = cWorld( )
        # setup controls
        self.setupControls( )
        # display title information
        self.title = OnscreenText( text = 'BOIDS.PY - SEEK & ARRIVE', fg = ( 1.0, 1.0, 1.0, 1.0), pos = ( -.98, .9 ), scale = 0.06 )
        # display copright information
        self.copyRight = OnscreenText( text = 'Copyright (C) 2007 David Lettier.', fg = ( 1.0, 1.0, 1.0, 1.0), pos = ( .98, -.98 ), scale = 0.05 )
        # display panda version text
        self.pandaVersion = OnscreenText( text = 'Panda Version 1.3.2', fg = ( 1.0, 1.0, 1.0, 1.0), pos = ( -1.18, -.98 ), scale = 0.04 )
        # display print debug button
        # this button calls the prntDebug function
        self.prntDebugButton = DirectButton( text = "Print Debug", relief = DGG.RAISED, scale = .1, pad = ( .5, .5 ), pos = Vec3( -1.0, 0.0, -.8 ), command = self.prntDebug )
    def setupControls( self ):
        # accept the esc key to exit application
        self.accept( 'escape', sys.exit )
    def prntDebug( self ):
        # destory debug button
        self.prntDebugButton.destroy( )
        # create new clear debug button
        # this button calls the clearDebug function
        self.clearDebugButton = DirectButton( text = "Clear Debug", relief = DGG.RAISED, scale = .1, pad = ( .5, .5 ), pos = Vec3( -1.0, 0.0, -.8 ), command = self.clearDebug )
        # create green target position text
        self.greenPos = OnscreenText( text = 'Green Target Pos: ' + str( int( self.world.target.getX( ) ) ) + ", " + str( int( self.world.target.getY( ) ) ) + ", " + str( int( self.world.target.getZ( ) ) ), fg = ( 1.0, 1.0, 1.0, 1.0), pos = ( -.8, .8 ), scale = 0.05, mayChange = True )
        # create blue boid position text
        self.bluePos = OnscreenText( text = 'Blue Boid (Arrive) Pos: ' + str( int( self.world.blueBoid.boidModel.getX( ) ) ) + ", " + str( int( self.world.blueBoid.boidModel.getY( ) ) ) + ", " + str( int( self.world.blueBoid.boidModel.getZ( ) ) ), fg = ( 1.0, 1.0, 1.0, 1.0), pos = ( -.8, .7 ), scale = 0.05, mayChange = True )
        # create green boid position text
        self.redPos = OnscreenText( text = 'Red Boid (Seek) Pos: ' + str( int( self.world.redBoid.boidModel.getX( ) ) ) + ", " + str( int( self.world.redBoid.boidModel.getY( ) ) ) + ", " + str( int( self.world.redBoid.boidModel.getZ( ) ) ), fg = ( 1.0, 1.0, 1.0, 1.0), pos = ( -.8, .6 ), scale = 0.05, mayChange = True )
        # add the update on screen text task
        taskMgr.add( self.updateOSTxt, 'updateOSTxt' )
    def updateOSTxt( self, Task ):
        # update green target position text
        self.greenPos.setText( 'Green Target Pos: ' + str( int( self.world.target.getX( ) ) ) + ", " + str( int( self.world.target.getY( ) ) ) + ", " + str( int( self.world.target.getZ( ) ) ) )
        # update blue boid position text
        self.bluePos.setText( 'Blue Boid (Arrive) Pos: ' + str( int( self.world.blueBoid.boidModel.getX( ) ) ) + ", " + str( int( self.world.blueBoid.boidModel.getY( ) ) ) + ", " + str( int( self.world.blueBoid.boidModel.getZ( ) ) ) )
        # update red boid position text
        self.redPos.setText( 'Red Boid (Seek) Pos: ' + str( int( self.world.redBoid.boidModel.getX( ) ) ) + ", " + str( int( self.world.redBoid.boidModel.getY( ) ) ) + ", " + str( int( self.world.redBoid.boidModel.getZ( ) ) ) )
        # call task next frame
        return Task.cont
    def clearDebug( self ):
        # destory button
        self.clearDebugButton.destroy( )
        # destory all debug on screen text
        self.greenPos.destroy( )
        self.bluePos.destroy( )
        self.redPos.destroy( )
        # remove task
        taskMgr.remove( 'updateOSTxt' )
        # create again the print debug button to start the cycle all over again
        self.prntDebugButton = DirectButton( text = "Print Debug", relief = DGG.RAISED, scale = .1, pad = ( .5, .5 ), pos = Vec3( -1.0, 0.0, -.8 ), command = self.prntDebug )
# create application instance
application = cApplication( )
#
if __name__ == '__main__': 
    run( ) # if this module is not being imported run the game