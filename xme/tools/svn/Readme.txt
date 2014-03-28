Subversion Utitity Files
========================

This folder contains the following files and folders:

config
------

This is a sample config file that should be used for configuring your SVN client.
It should be placed at the following location, depending on your operating system:

- Linux:
  ~/.subversion

- Windows:
  %APPDATA%\Subversion

The file will do the following:

- Enable SVN autoprops. Autoprops is a feature that will automatically add certain
  flags to all files that are added to version control. In the case of this config
  file, we ensure that all text files have the svn:keywords property set (for
  automatic keyword expansion) and that the svn:eol-style property is set to LF
  (Linux line endings) for those files. Furthermore, we set the svn:mime-type of
  some common file types to meaningful values.

For more information about Subversion and the config file, please read the relevant
sections in the latest version of the Redbean Subversion book [1].

hooks
-----

This folder contains server-side hook files that are (to be) installed on the SVN
server. The scripts currently ensure that svn:keywords and svn:eol-style properties
are set correctly on text files.

--
[1] <http://svnbook.red-bean.com/>