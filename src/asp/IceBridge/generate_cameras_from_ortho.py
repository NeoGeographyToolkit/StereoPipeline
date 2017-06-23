#!/usr/bin/env python


# Generate camera files and single channel image files from the ortho images

import os, sys, optparse, tempfile, shutil


def main(argsIn):

    try:
        usage = "usage: generate_cameras_from_ortho.py <raw_folder> <ortho_folder> <camera_file>  <output_folder> [--help]\n  "
        parser = optparse.OptionParser(usage=usage)

        # Note: The same reference DEM may not work for all images! This better be left blank.
        parser.add_option('--reference-dem', action='store', default="", dest='referenceDem',  
                          type='string', help='Get the heights above the datum from this DEM.')
        
        (options, args) = parser.parse_args(argsIn)

        if len(args) < 5:
            print usage
            return 0

        rawFolder    = args[1]
        orthoFolder  = args[2]
        cameraFile   = args[3]
        outputFolder = args[4]

    except optparse.OptionError, msg:
        raise Exception(msg)
    
    # Check the inputs
    if not os.path.exists(rawFolder):
        print 'Raw input folder does not exist!'
        return 0
    if not os.path.exists(orthoFolder):
        print 'Ortho input folder does not exist!'
        return 0
    if not os.path.exists(outputFolder):
        os.mkdir(outputFolder)

    tempImageDir = tempfile.mkdtemp()

    # Loop through the ortho files and find matches in the raw files
    # - The ortho file names have more info so best to start with them.
    orthoFiles = os.listdir(orthoFolder)

    numWritten = 0
    for f in orthoFiles:
        # Skip other files
        if '.tif' not in f:
            continue
        
        # Generate the path to the raw file
        parts     = f.split('_')
        count     = parts[2]
        date      = parts[3]
        time      = parts[4].replace('.tif','')
        rawString = date[0:4]+'_'+date[4:6]+'_'+date[6:8]+'_'+count+'.JPG'
        rawPath   = os.path.join(rawFolder,   rawString)
        orthoPath = os.path.join(orthoFolder, f)

        if not os.path.exists(rawPath):
            print 'Could not find matching raw file for ortho file: ' + f
            continue

        # Set up the output paths
        outputString = count+'_'+date+'_'+time+'.tif'
        outputImagePath  = os.path.join(outputFolder,outputString)
        if (options.referenceDem != "") :
            outputCameraPath = outputImagePath.replace('.tif','.dem.tsai')
        else:
            outputCameraPath = outputImagePath.replace('.tif','.tsai')

        if os.path.exists(outputCameraPath):
            print 'Output file ' + outputCameraPath + ' already exists, skipping it.'
            continue

            # DEBUG CODE
            #tempCameraPath = outputCameraPath + '.bak.tsai'
            #shutil.move(outputCameraPath, tempCameraPath)
            
            ## Convert to a TSAI format camera model
            ## - This is to make bundle adjustment faster!
            #cmd = 'convert_pinhole_model -o ' + outputCameraPath +' '+ outputImagePath +' '+ tempCameraPath
            #print cmd
            #os.system(cmd)
            #continue
            

        # Generate the single channel image file
        cmd = ('gdal_translate -b 1 '+rawPath+'  '+outputImagePath)
        print cmd
        os.system(cmd)
        
        # Generate a single channel copy of the ortho image
        tempOrthoPath  = os.path.join(tempImageDir, f)
        cmd = ('gdal_translate -b 1 '+orthoPath+' '+tempOrthoPath)
        print cmd
        os.system(cmd)

        
        # Generate the camera file
        tempCameraPath  = os.path.join(tempImageDir, count+'.tsai')
        cmd = ('ortho2pinhole ' +outputImagePath+' '+tempOrthoPath+' '+cameraFile+' '+tempCameraPath)
        if (options.referenceDem != "") :
            cmd += ' --reference-dem ' + options.referenceDem
        print cmd
        os.system(cmd)
        
        # Convert to a TSAI format camera model
        # - This is to make bundle adjustment faster!
        cmd = 'convert_pinhole_model -o ' + outputCameraPath +' '+ outputImagePath +' '+ tempCameraPath
        print cmd
        os.system(cmd)

        if os.path.exists(outputCameraPath):
            numWritten += 1
            tempGcpPath = tempCameraPath + ".gcp"
            outputGcpPath = outputCameraPath + ".gcp"
            print("Writing GCP file: " + outputGcpPath)
            shutil.copyfile(tempGcpPath, outputGcpPath)

    # Clean up the temporary ortho images
    os.system('rm -rf '+tempImageDir)
    
    print 'Wrote ' + str(numWritten) +' output files.'

# Run main function if file used from shell
if __name__ == "__main__":
    sys.exit(main(sys.argv))


