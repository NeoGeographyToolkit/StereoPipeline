============
Contributing
============

Contributions are welcome, and they are greatly appreciated! Every
little bit helps, and credit will always be given.

You can contribute to ASP in many ways:

Types of Contributions
----------------------

Report Bugs or Ask for Features via Issues
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We want to hear from you!  You can report bugs, ask for new features,
or just raise issues or concerns via logging an `Issue via our GitHub
repo <https://github.com/NeoGeographyToolkit/StereoPipeline/issues>`_.


Fix Bugs or Implement Features
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Look through the GitHub Issues for bugs to fix or features to
implement.  If anything looks tractable to you, work on it.  Most (if
not all) pull requests should be based on an Issue, so if you're
thinking about doing some coding on a topic that isn't covered in an
Issue, please author one so you can get some feedback while you work
on your pull request.

Write Documentation
~~~~~~~~~~~~~~~~~~~

ASP could always use more documentation, whether as part of the
official docs, on the web in blog posts, articles, and such.

Submit Feedback
~~~~~~~~~~~~~~~

The best way to send feedback is to file an `Issue
<https://github.com/NeoGeographyToolkit/StereoPipeline/issues>`_.

Get Started!
------------

Ready to contribute? 

You'll need to follow the instructions for building ASP from source,
which can be found in the INSTALLGUIDE.rst file or the Installation
chapter of the documentation.

1. Fork the `StereoPipeline` repo on GitHub.

2. Clone your fork locally::

    $ git clone git@github.com:your_name_here/StereoPipeline.git

3. Create a branch for local development::

    $ git checkout -b name-of-your-bugfix-or-feature

   Now you can make your changes locally.

4. When you're done making changes, check that your changes pass a run
   of `make gtest_all` (though note that the unit tests have been
   broken recently so this won't work).

5. Commit your changes and push your branch to GitHub::

    $ git add .
    $ git commit -m "Your detailed description of your changes."
    $ git push origin name-of-your-bugfix-or-feature

6. Submit a `pull request <https://github.com/NeoGeographyToolkit/StereoPipeline/pulls>`_.


Pull Request Guidelines
-----------------------

Before you submit a pull request, check that it meets these guidelines:

1. The pull request should include tests.
2. If the pull request adds functionality, the docs should be updated. 
   Add the feature to the list in NEWS.rst and potentially update the
   README.rst or other documentation files.

What to expect
--------------

Our development of ASP is neither continuous, nor as well-funded as we
might like, and it is entirely possible that when you submit a PR
(pull request), none of us will have the time to evaluate or integrate
your PR.  If we don't, we'll try and communicate that with you via the
PR.

For large contributions, it is likely that you, or your employer,
will be retaining your copyrights, but releasing the contributions
via an open-source license.  It must be compatible with the Apache-2
license that ASP is distributed with, so that we can redistribute
that contribution with ASP, give you credit, and make ASP even
better!  Please contact us if you have a contribution of that nature, 
so we can be sure to get all of the details right.

For smaller contributions, where you (or your employer) are not
concerned about retaining copyright (but we will give you credit!),
you will need to fill out a Contributor License Agreement (CLA)
if we plan to accept your PR.  The CLA assigns your copyright in
your contribution to NASA, so that our NASA copyright statement
remains true:

    Copyright (c) YEAR, United States Government as represented by the 
    Administrator of the National Aeronautics and Space Administration.
    All rights reserved.

There is an `Individual CLA <https://github.com/NeoGeographyToolkit/StereoPipeline/blob/master/docs/ASP_Individual_CLA.pdf>`_ and a `Corporate CLA
<https://github.com/NeoGeographyToolkit/StereoPipeline/blob/master/docs/ASP_Corporate_CLA.pdf>`_.

ASP People
----------

- An ASP **Contributor** is any individual creating or commenting
  on an issue or pull request.  Anyone who has authored a PR that was
  merged should be listed in the AUTHORS.rst file.  

- An ASP **Committer** is a subset of contributors, typically NASA
  employees or contractors, who have been given write access to the
  repository.
