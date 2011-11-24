/*
    Copyright 2010 Chris Desjardins - cjd@chrisd.info

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
package ball.Paper;
import javax.microedition.khronos.opengles.GL10;
import javax.microedition.khronos.egl.EGLConfig;
import java.nio.FloatBuffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import android.util.Log;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.GLWallpaperService;

/**
 * @author chrisd
 *
 */
public class BallPaper extends GLWallpaperService 
{
    public BallPaper() 
    {
        super();
        /*android.os.Debug.waitForDebugger();*/
    }
    
    @Override
    public void onCreate() 
    {
        super.onCreate();
    }
    
    @Override
    public void onDestroy() 
    {
        super.onDestroy();
    }

    @Override
    public Engine onCreateEngine() 
    {
        return new BallPaperEngine();
    }

    /*************************************************************
     * BallPaperEngine
     *************************************************************/
    class BallPaperEngine extends GLEngine 
    { 
        private final SensorEventListener mAccelListener = new SensorEventListener() 
        {
            public void onAccuracyChanged(Sensor sensor, int accuracy) 
            {
                
            }
            /*************************************************************
             * onSensorChanged - Read input from the mag, and accel
             *************************************************************/
            public void onSensorChanged(SensorEvent event) 
            {
                if (event.accuracy != SensorManager.SENSOR_STATUS_UNRELIABLE)
                {
                    if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) 
                    {
                        renderer.setAccel(event.values);
                    }
                    if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
                    {
                        renderer.setMags(event.values);
                    }
                }
            }
        };
        /*************************************************************
         * BallPaperEngine - Creates a new render engine.
         *************************************************************/
        public BallPaperEngine() 
        { 
            super(); 
            // handle prefs, other initialization 
            renderer = new MyRenderer(); 
            setRenderer(renderer); 
            setRenderMode(RENDERMODE_CONTINUOUSLY);
            mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        }
        
        @Override
        public void onDestroy() 
        {
            super.onDestroy();
            mSensorManager.unregisterListener(mAccelListener);
        }
        /*************************************************************
         * onVisibilityChanged - Callback for changes in visibility state.
         *************************************************************/
        @Override
        public void onVisibilityChanged(boolean visible) 
        {
            super.onVisibilityChanged(visible);
            if (visible) 
            {
                Sensor accel = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
                Sensor mag = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
                if (mSensorManager.registerListener(mAccelListener, accel, SensorManager.SENSOR_DELAY_NORMAL) == false)
                {
                    Log.v("chrisd", "No accel");
                }
                if (mSensorManager.registerListener(mAccelListener, mag, SensorManager.SENSOR_DELAY_NORMAL) == false)
                {
                    Log.v("chrisd", "No mag");
                }
            }
            else
            {
                mSensorManager.unregisterListener(mAccelListener);
            }
        }
        MyRenderer renderer; 
        private SensorManager mSensorManager = null;
    }
    
    /*************************************************************
     * CJDSphere - Code to render a sphere in OpenGL.
     *************************************************************/

    class CJDSphere 
    {
        /*************************************************************
         * draw - Draws the actual sphere, with normal vectors for lighting.
         *************************************************************/
        public void draw(GL10 gl)
        {
            int j;
            gl.glEnableClientState(GL10.GL_NORMAL_ARRAY);
            gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
            for (j = 0; j < (mResolution / 2); j++)
            {
                gl.glNormalPointer(GL10.GL_FLOAT, 0, mNorms[j]);
                gl.glVertexPointer(3, GL10.GL_FLOAT, 0, mVerts[j]);
                gl.glDrawArrays(GL10.GL_TRIANGLE_STRIP, 0, mVerts[j].limit() / 3);
            }
        }
        /*************************************************************
         * initBuffers - Allocated memory necessary for drawing a 
         * sphere of the given resolution.
         *************************************************************/
        private void initBuffers(int resolution)
        {
            int j;
            mN = new ByteBuffer[resolution / 2];
            mV = new ByteBuffer[resolution / 2];
            mNorms = new FloatBuffer[resolution / 2];
            mVerts = new FloatBuffer[resolution / 2];
            for (j = 0; j < (resolution / 2); j++)
            {
                mN[j] = ByteBuffer.allocateDirect((resolution + 1) * 4 * 6);
                mV[j] = ByteBuffer.allocateDirect((resolution + 1) * 4 * 6);
    
                mN[j].order(ByteOrder.nativeOrder());
                mV[j].order(ByteOrder.nativeOrder());
                
                mNorms[j] = mN[j].asFloatBuffer();
                mVerts[j] = mV[j].asFloatBuffer();
            }
        }
        /*************************************************************
         * CJDSphere - Setsup buffers and creates the vertices for the 
         * sphere given the radius and resolution.
         * Uses the trigonometric formula for a sphere.
         *************************************************************/
        public CJDSphere(float radius, int resolution)
        {
            int i, j;
            float theta1, theta2, theta3;
            float x,y,z;
            mResolution = resolution;
            initBuffers(resolution);
            
            for(j = 0; j < resolution / 2; j++) 
            {
                theta1 = (float)(j * (Math.PI * 2.0f) / resolution - (Math.PI / 2));
                theta2 = (float)((j + 1) * (Math.PI * 2.0f) / resolution - (Math.PI / 2));
                
                for(i = 0; i <= resolution; i++) 
                {
                    theta3 = (float)(i * (Math.PI * 2.0f) / resolution);
                    
                    x = (float)(Math.cos(theta2) * Math.cos(theta3));
                    y = (float)(Math.sin(theta2));
                    z = (float)(Math.cos(theta2) * Math.sin(theta3));
                    mNorms[j].put(x);
                    mNorms[j].put(y);
                    mNorms[j].put(z);
                    mVerts[j].put(radius * x);
                    mVerts[j].put(radius * y);
                    mVerts[j].put(radius * z);

                    x = (float)(Math.cos(theta1) * Math.cos(theta3));
                    y = (float)(Math.sin(theta1));
                    z = (float)(Math.cos(theta1) * Math.sin(theta3));
                    mNorms[j].put(x);
                    mNorms[j].put(y);
                    mNorms[j].put(z);
                    mVerts[j].put(radius * x);
                    mVerts[j].put(radius * y);
                    mVerts[j].put(radius * z);
                }
                mVerts[j].position(0);
                mNorms[j].position(0);
            }
        }
        int mResolution;
        ByteBuffer mN[];
        ByteBuffer mV[];
        FloatBuffer mNorms[];
        FloatBuffer mVerts[];
    }
    
    /*************************************************************
     * MyRenderer
     *************************************************************/

    class MyRenderer implements Renderer
    {
        MyRenderer() 
        {
            super();
            mysphere = new CJDSphere(0.5f, 62);
        }
        /*************************************************************
         * setupLights - Configures the OpenGL lights which will nicely
         * illuminate the sphere.
         *************************************************************/
        public void setupLights(GL10 gl) 
        {
            gl.glLightfv(GL10.GL_LIGHT1, GL10.GL_AMBIENT, mLightAmbient, 0);
            gl.glLightfv(GL10.GL_LIGHT0, GL10.GL_DIFFUSE, mLightDiffuse, 0);
            gl.glLightfv(GL10.GL_LIGHT0, GL10.GL_POSITION, mLightPosition, 0);
            gl.glEnable(GL10.GL_LIGHT0);
            gl.glEnable(GL10.GL_LIGHT1);
            gl.glEnable(GL10.GL_LIGHTING);
            
            gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_AMBIENT, mMatAmbient);
            gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_DIFFUSE, mMatDiffuse);
            gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_SPECULAR, mMatSpecular);
            gl.glMaterialf(GL10.GL_FRONT_AND_BACK, GL10.GL_SHININESS, mMatLowShininess);
        }
        /*************************************************************
         * onDrawFrame - Draws, lights, and translates the whole scene.
         *************************************************************/
        @Override
        public void onDrawFrame(GL10 gl) 
        {
            gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);

            gl.glMatrixMode(GL10.GL_MODELVIEW);
            gl.glLoadIdentity();
            gl.glTranslatef(mLocation[0], mLocation[1], mLocation[2]);
            setupLights(gl);

            mysphere.draw(gl);

            computeUp();
            computeSpeed();
            
            moveIt(0, -mWidth / 200.f, mWidth / 200.f);
            moveIt(1, -3.0f, 3.0f);
            moveIt(2, -8.0f, -3.0f);
        }
        /*************************************************************
         * onSurfaceChanged 
         *************************************************************/
        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) 
        {
            float ratio;
            mWidth = width;
            gl.glViewport(0, 0, width, height);
            ratio = (float) width / height;
            Log.v("chrisd", "surface change w " + width + " h " + height + " r " + ratio);
            gl.glMatrixMode(GL10.GL_PROJECTION);
            gl.glLoadIdentity();
            gl.glFrustumf(-ratio, ratio, -1, 1, 1, 10);
        }
        /*************************************************************
         * onSurfaceCreated
         *************************************************************/
        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) 
        {
            gl.glDisable(GL10.GL_DITHER);
            gl.glEnable(GL10.GL_NORMALIZE);
            gl.glHint(GL10.GL_PERSPECTIVE_CORRECTION_HINT, GL10.GL_FASTEST);
            gl.glClearColor(0,0,0,0);
            gl.glShadeModel(GL10.GL_SMOOTH);
            gl.glEnable(GL10.GL_DEPTH_TEST);
        }
        /*************************************************************
         * computeSpeed - sets the speed for the x,y,z directions and
         * clamps to the speed limit.
         *************************************************************/
        public void computeSpeed()
        {
            setSpeed(0);
            setSpeed(1);
            setSpeed(2);
        }
        /*************************************************************
         * computeUp - Determines the UP vector, the up vector always points
         * away from gravity, this is used to cancel out the effect of
         * gravity on the accelerometer. When the android device is at rest
         * the accelerometer reads 9.8 m/s**2 "down" so the up vector
         * can be used to add 9.8 m/s**2 so that when the accelerometer
         * is at rest the result is that all accelerations are 0.
         *************************************************************/
        public void computeUp()
        {
            SensorManager.getRotationMatrix(mRotation, null, mAccel, mMags);
            // Correct if screen is in Landscape 
            /*SensorManager.remapCoordinateSystem(rotation, SensorManager.AXIS_X, SensorManager.AXIS_Z, outR);*/
            SensorManager.getOrientation(mRotation, mOrientation);

            mUpVec[0] = (float)-((Math.sin(mOrientation[2]) * Math.cos(mOrientation[1])) * SensorManager.GRAVITY_EARTH);
            mUpVec[1] = (float)-(Math.sin(mOrientation[1]) * SensorManager.GRAVITY_EARTH);
            mUpVec[2] = (float)((Math.cos(mOrientation[2]) * Math.cos(mOrientation[1])) * SensorManager.GRAVITY_EARTH);
        }
        public void setAccel(float accel[])
        {
            System.arraycopy(accel, 0, mAccel, 0, 3);
        }
        public void setMags(float mags[])
        {
            System.arraycopy(mags, 0, mMags, 0, 3);
        }
        public void setSpeed(int i)
        {
            final float max_speed = 1.0f;
            if ((Math.abs(mAccel[i]) - Math.abs(mUpVec[i])) > 1.0f)
            {
                mSpeed[i] = mAccel[i] - mUpVec[i];
            }
            if (mSpeed[i] > max_speed)
            {
                mSpeed[i] = max_speed;
            }
            if (mSpeed[i] < -max_speed)
            {
                mSpeed[i] = -max_speed;
            }
        }
        /*************************************************************
         * reduceSpeed - No fancy forumula with the coefficient of drag or
         * anything, just make the sphere go slower in specified direction.
         *************************************************************/
        public void reduceSpeed(int i, float max, float min, float step)
        {
            if ((Math.abs(mSpeed[i]) <= max) && (Math.abs(mSpeed[i]) > min))
            {
                if (mSpeed[i] < 0.0f)
                {
                    mSpeed[i] += step;
                }
                else
                {
                    mSpeed[i] -= step;                    
                }
            }
        }
        /*************************************************************
         * moveIt - Based on input from the accelerometer, give the
         * sphere a speed in a specified dimention.
         *************************************************************/
        public void moveIt(int i, float bound0, float bound1) 
        {
            mLocation[i] += mSpeed[i];
            
            if ((mLocation[i] < bound0) || (mLocation[i] > bound1))
            {
                mSpeed[i] = -mSpeed[i];
            }
            if (mLocation[i] < bound0)
            {
                mLocation[i] = bound0;
            }
            if (mLocation[i] > bound1)
            {
                mLocation[i] = bound1;
            }
            if (Math.abs(mSpeed[i]) <= 0.001f)
            {
                mSpeed[i] = 0.0f;
            }
            reduceSpeed(i, 10.0f, 0.5f, 0.01f);
            reduceSpeed(i, 0.5f, 0.2f, 0.005f);
            reduceSpeed(i, 0.2f, 0.001f, 0.001f);
        }
        float mMatAmbientf[] = new float[]{0.1f, 0.1f, 0.1f, 1.0f};
        FloatBuffer mMatAmbient = FloatBuffer.wrap(mMatAmbientf);
        float mMatDiffusef[] = new float[]{0.1f, 0.5f, 0.8f, 1.0f};
        FloatBuffer mMatDiffuse = FloatBuffer.wrap(mMatDiffusef);
        float mMatSpecularf[] = new float[]{1.0f, 1.0f, 1.0f, 1.0f};
        FloatBuffer mMatSpecular = FloatBuffer.wrap(mMatSpecularf);
        float mMatLowShininess = 10.0f;
        
        float mLightDiffuse[]    = {20.0f, 0.0f, 0.0f, 0.0f};
        float mLightAmbient[]    = {5.0f, 0.0f, 0.0f, 1.0f};
        float mLightPosition[]   = {5.0f, 5.0f, 20.0f, 0.0f};
        
        float[] mRotation = new float[16];
        float[] mOrientation = new float[3];

        float mMags[] = {0.0f, 0.0f, 0.0f};
        float mAccel[] = {0.0f, 0.0f, SensorManager.GRAVITY_EARTH};
        
        float mLocation[] = { 0.0f, 0.0f, -6.0f};
        float mSpeed[] = {0.0f, 0.0f, 0.0f};
        float mUpVec[] = {0.0f, 0.0f, 1.0f};
        float mWidth;
        int mRenderCnt[] = {0, 0, 0};
        CJDSphere mysphere; 
    }
}
