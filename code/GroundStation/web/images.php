<?
$directory = 'out/images';    //where the gallery images are located
 
$allowed_types=array('jpg','jpeg','gif','png');    //allowed image types
 
$file_parts=array();
$ext='';
$title='';
$i=0;
 
//try to open the directory
$dir_handle = @opendir($directory) or die("There is an error with your image directory!");
 
$files = array();
while (false !== ($file = readdir($dir_handle))) {
       if ($file != "." && $file != ".." && filesize($directory . "/" . $file) > 0) {
          $files[filemtime($directory . "/" . $file)] = $file;
       }
   }
closedir($dir_handle);
ksort($files);


# while ($file = readdir($dir_handle))    //traverse through the files
$pic_count = 1;
foreach (array_reverse($files) as $file)
{
if($file=='.' || $file == '..') continue;    //skip links to the current and parent directories

if ($pic_count > 10) break;
++$pic_count;
 
$file_parts = explode('.',$file);    //split the file name and put each part in an array
$ext = strtolower(array_pop($file_parts));    //the last element is the extension
 
$title = implode('.',$file_parts);    //once the extension has been popped out, all that is left is the file name
$title = htmlspecialchars($title);    //make the filename html-safe to prevent potential security issues
 
$nomargin='';

echo "<ul id=\"imagelist\">\n";
if(in_array($ext,$allowed_types))    //if the extension is an allowable type
{
if(($i+1)%4==0) $nomargin='nomargin';    //the last image on the row is assigned the CSS class "nomargin"
$cmd = 'convert ' . $directory . '/' . $file . ' -geometry 160x120 ' . $directory . '/thumbnails/' . $file; 

$thumbnail = $directory . '/thumbnails/' . $file;

if ( ! file_exists($thumbnail) || filemtime($thumbnail) < filemtime($directory . '/' . $file))
{
`$cmd`;
}

echo "  <li>\n";
echo '<a title="'.$title.'" href="'.$directory.'/'.$file.'" target="_blank"><img src="'.$directory.'/thumbnails/'.$file.'"/>'.$title.'</a>';
echo "  </li>\n";
 
$i++;    //increment the image counter
}
}
echo "</ul>\n";
 
# closedir($dir_handle);    //close the directory

?>
