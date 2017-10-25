///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Oct 23 2017)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __PROBVIEWER_B_H__
#define __PROBVIEWER_B_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
class ImagePanel;

#include <wx/string.h>
#include <wx/checkbox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// Class ProbViewer_B
///////////////////////////////////////////////////////////////////////////////
class ProbViewer_B : public wxFrame 
{
	private:
	
	protected:
		wxBoxSizer* bSizer2;
		wxCheckBox* log_checkbox_;
		wxCheckBox* rescale_checkbox_;
		ImagePanel* image_panel_;
		wxStaticText* m_staticText2;
		wxStaticText* min_text_;
		wxStaticText* m_staticText4;
		wxStaticText* max_text_;
		wxStaticText* m_staticText6;
		wxStaticText* current_prob_text_;
		wxStaticText* m_staticText8;
		wxStaticText* curr_log_text_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onCheck( wxCommandEvent& event ) { event.Skip(); }
		virtual void onMouseMove( wxMouseEvent& event ) { event.Skip(); }
		
	
	public:
		
		ProbViewer_B( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("ProbViewer"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,-1 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		
		~ProbViewer_B();
	
};

#endif //__PROBVIEWER_B_H__
