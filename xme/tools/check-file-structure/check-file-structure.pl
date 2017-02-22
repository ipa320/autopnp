#!/bin/perl
#
# Copyright (c) 2012-2013, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: check-file-structure.pl 6630 2014-02-06 13:30:30Z geisinger $
#
# File:
#         Static file structure checker script.
#

use strict;

# Settings
my @rootDirs = ('../../xme', '../../tests', '../../examples');
#my @rootDirs = ('test');
my $templatesDir = 'templates';
my $ignores = '^(?:\.svn|_svn|build|_CPack_Packages|.*\.zip|.*\.pdf)$';
my $verbose = 0;

# Variables
my %templates;
my $warnCount = 0;
my $errorCount = 0;

&loadTemplates();

foreach(@rootDirs)
{
	&processDir($_);
}

sub printVerbose
{
	print @_ if $verbose;
}

sub loadTemplates
{
	my $dir = $templatesDir;

	opendir(DIR, $dir);
	foreach (readdir(DIR))
	{
		if (!-d "$dir/$_" && $_ =~ /^(.+)\.tmpl$/)
		{
			my $name = $1;
			$name =~ s/\./\\./g;
			$name =~ s/\+/.*/g; # wildcard '*'
			$name =~ s/!/./g; # wildcard '?'
			$name = "^$name\$";

			if (!open(FILE, "<$dir/$_"))
			{
				print STDERR "Unable to open template file '$dir/$_', skipping!\n";
				next;
			}
			my @lines = <FILE>;
			close(FILE);

			foreach(@lines)
			{
				$_ =~ s/[\r\n]+$//;
			}

			$templates{$name} = join("\n", @lines);

			&printVerbose("Found template '$dir/$_' for pattern '$name'\n");
		}
	}
	closedir(DIR);
}

sub processDir
{
	my $dir = $_[0];

	opendir(DIR, $dir);
	foreach (readdir(DIR))
	{
		next if ($_ =~ /$ignores/);

		if (-d "$dir/$_")
		{
			if ($_ ne '.' && $_ ne '..')
			{
				# Recurse
				&processDir("$dir/$_");
			}
		}
		else
		{
			# Process file
			&processFile("$dir/$_");
		}
	}
	closedir(DIR);
}

sub findDifference
{
	my @a = split(/\n/, $_[0]);
	my @b = split(/\n/, $_[1]);
	my $p = $_[2];

	while (scalar(@a) > 0 && scalar(@b) > 0 && $a[0] eq $b[0])
	{
		shift(@a);
		shift(@b);
		$p++;
	}

	my $a = scalar(@a) > 0 ? $a[0] : '';
	my $b = scalar(@b) > 0 ? $b[0] : '';

	return ($a, $b, $p);
}

sub formatWhitespace
{
	my $a = $_[0];

	# Replace space by "·" and tab by "··­>"
	$a =~ s/ /·/gs;
	$a =~ s/\t/··­>/gs;

	return $a;
}

sub processFile
{
	my $filename = $_[0];

	&printVerbose("Checking '$filename' ... ");

	my @keys = keys(%templates);
	my $matchingTemplates = 0;
	my $fileErrorCount = 0;
	for (my $i=0; $i<scalar(@keys); $i++)
	{
		next unless ($_ =~ /$keys[$i]/);
		$matchingTemplates++;

		if (!open(FILE, "<$filename"))
		{
			print STDERR "Unable to open file '$filename', skipping!\n";
			next;
		}
		my @file = <FILE>;
		close(FILE);

		foreach(@file)
		{
			$_ =~ s/[\r\n]+$/\n/;
		}

		my $file = join('', @file);

		my $tmpl = $templates{$keys[$i]};

		my $lineNum = 1;

		while ($tmpl =~ /^(.*?)<<(.*?)>>'([^']*)'(.*)$/s)
		{
			my $pre = $1;
			my $in = $2;
			my $msg = $3;
			my $post = $4;

			if ($file !~ /^\Q$pre\E(.*)$/s)
			{
				if (0 == $fileErrorCount++) { &printVerbose("error\n"); }

				my $found = substr($file, 0, length($pre));
				my $preFmt = $pre;
				my $pos = $lineNum;

				($found, $preFmt, $pos) = &findDifference($found, $preFmt, $pos);
				$found = &formatWhitespace($found);
				$preFmt = &formatWhitespace($preFmt);

				$found =~ s/\n/\n  > /g;
				$preFmt =~ s/\n/\n  > /g;

				$errorCount++;
				print STDERR <<END;
$filename($pos): Error: Non-conformant structure!
 >> Found:
  > $found
 >> Expected:
  > $preFmt

END
				return;
			}
			else
			{
				$file = $1;
				$lineNum += ($pre =~ tr/\n/\n/);
			}

			if ($file !~ /^($in)(.*)$/s)
			{
				if (0 == $fileErrorCount++) { &printVerbose("error\n"); }

				my $num = 100;

				my $found = substr($file, 0, $num);
				my $inFmt = $in;

				$found = &formatWhitespace($found);
				$inFmt = &formatWhitespace($inFmt);

				$found =~ s/\n/\n  > /g;
				$inFmt =~ s/\n/\n  > /g;

				$errorCount++;
				print STDERR <<END;
$filename($lineNum): Error: $msg
 >> Found (first $num characters):
  > $found
 >> Expected pattern:
  > $inFmt

END
				return;
			}
			else
			{
				$file = $2;
				$lineNum += ($1 =~ tr/\n/\n/);
				#print "FILE MATCHES '$in', REST='$file'\n\n\n";
			}

			$tmpl = $post;
		}

		if ($file !~ /^\Q$tmpl\E/s)
		{
			if (0 == $fileErrorCount++) { &printVerbose("error\n"); }

			my $found = substr($file, 0, length($tmpl));
			my $tmplFmt = $tmpl;
			my $pos = $lineNum;

			($found, $tmplFmt, $pos) = &findDifference($found, $tmplFmt, $pos);
			$found = &formatWhitespace($found);
			$tmplFmt = &formatWhitespace($tmplFmt);

			$found =~ s/\n/\n  > /g;
			$tmplFmt =~ s/\n/\n  > /g;

			$errorCount++;
			print STDERR <<END;
$filename($pos): Error: Non-conformant structure!
 >> Found:
  > $found
 >> Expected:
  > $tmplFmt

END
			return;
		}
	}

	if (0 == $fileErrorCount)
	{
		if (1 != $matchingTemplates)
		{
			&printVerbose("warn\n");
			$warnCount++;
			print STDERR "$filename: Warning: $matchingTemplates matching template(s)!\n";
		}
		else
		{
			&printVerbose("ok\n");
		}
	}
}

print "$errorCount error(s), $warnCount warning(s)";
