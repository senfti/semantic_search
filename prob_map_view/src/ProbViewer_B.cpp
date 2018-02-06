///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Oct 23 2017)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "prob_map_view/ImagePanel.h"

#include "ProbViewer_B.h"

///////////////////////////////////////////////////////////////////////////

ProbViewer_B::ProbViewer_B( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 480,320 ), wxDefaultSize );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxVERTICAL );
	
	bSizer2 = new wxBoxSizer( wxHORIZONTAL );
	
	log_checkbox_ = new wxCheckBox( this, wxID_ANY, wxT("Show in Log"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer2->Add( log_checkbox_, 0, wxALL, 5 );
	
	rescale_checkbox_ = new wxCheckBox( this, wxID_ANY, wxT("Scale to max"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer2->Add( rescale_checkbox_, 0, wxALL, 5 );
	
	
	bSizer3->Add( bSizer2, 0, wxEXPAND, 5 );
	
	image_panel_ = new ImagePanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	image_panel_->SetMinSize( wxSize( 320,320 ) );
	
	bSizer3->Add( image_panel_, 1, wxEXPAND | wxALL, 5 );
	
	
	bSizer1->Add( bSizer3, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxVERTICAL );
	
	wxGridSizer* gSizer1;
	gSizer1 = new wxGridSizer( 0, 2, 0, 0 );
	
	gSizer1->SetMinSize( wxSize( 150,-1 ) ); 
	m_staticText2 = new wxStaticText( this, wxID_ANY, wxT("Min:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	gSizer1->Add( m_staticText2, 0, wxALL, 5 );
	
	min_text_ = new wxStaticText( this, wxID_ANY, wxT("0.0"), wxDefaultPosition, wxDefaultSize, 0 );
	min_text_->Wrap( -1 );
	gSizer1->Add( min_text_, 0, wxALL, 5 );
	
	m_staticText4 = new wxStaticText( this, wxID_ANY, wxT("Max:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4->Wrap( -1 );
	gSizer1->Add( m_staticText4, 0, wxALL, 5 );
	
	max_text_ = new wxStaticText( this, wxID_ANY, wxT("1.0"), wxDefaultPosition, wxDefaultSize, 0 );
	max_text_->Wrap( -1 );
	gSizer1->Add( max_text_, 0, wxALL, 5 );
	
	m_staticText6 = new wxStaticText( this, wxID_ANY, wxT("Curr:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText6->Wrap( -1 );
	gSizer1->Add( m_staticText6, 0, wxALL, 5 );
	
	current_prob_text_ = new wxStaticText( this, wxID_ANY, wxT("0.5"), wxDefaultPosition, wxDefaultSize, 0 );
	current_prob_text_->Wrap( -1 );
	gSizer1->Add( current_prob_text_, 0, wxALL, 5 );
	
	m_staticText8 = new wxStaticText( this, wxID_ANY, wxT("log(Curr):"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText8->Wrap( -1 );
	gSizer1->Add( m_staticText8, 0, wxALL, 5 );
	
	curr_log_text_ = new wxStaticText( this, wxID_ANY, wxT("0.0"), wxDefaultPosition, wxDefaultSize, 0 );
	curr_log_text_->Wrap( -1 );
	gSizer1->Add( curr_log_text_, 0, wxALL, 5 );
	
	
	bSizer6->Add( gSizer1, 0, wxEXPAND, 5 );
	
	save_button_ = new wxButton( this, wxID_ANY, wxT("Save All"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer6->Add( save_button_, 0, wxALL, 5 );
	
	exit_button_ = new wxButton( this, wxID_ANY, wxT("Exit"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer6->Add( exit_button_, 0, wxALL, 5 );
	
	
	bSizer5->Add( bSizer6, 1, wxEXPAND, 5 );
	
	
	bSizer1->Add( bSizer5, 0, wxEXPAND, 5 );
	
	
	this->SetSizer( bSizer1 );
	this->Layout();
	bSizer1->Fit( this );
	
	this->Centre( wxBOTH );
	
	// Connect Events
	log_checkbox_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ProbViewer_B::onCheck ), NULL, this );
	rescale_checkbox_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ProbViewer_B::onCheck ), NULL, this );
	image_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( ProbViewer_B::onMouseMove ), NULL, this );
	save_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ProbViewer_B::saveAll ), NULL, this );
	exit_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ProbViewer_B::exit ), NULL, this );
}

ProbViewer_B::~ProbViewer_B()
{
	// Disconnect Events
	log_checkbox_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ProbViewer_B::onCheck ), NULL, this );
	rescale_checkbox_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ProbViewer_B::onCheck ), NULL, this );
	image_panel_->Disconnect( wxEVT_MOTION, wxMouseEventHandler( ProbViewer_B::onMouseMove ), NULL, this );
	save_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ProbViewer_B::saveAll ), NULL, this );
	exit_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ProbViewer_B::exit ), NULL, this );
	
}
